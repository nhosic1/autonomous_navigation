#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class LocalizationInitializer : public rclcpp::Node
{
public:
    LocalizationInitializer() : Node("localization_initializer_node")
    {
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&LocalizationInitializer::initial_pose_callback, this, std::placeholders::_1));

        reset_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/reset_pose", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autonomous_vehicle/imu_sensor/imu",
            rclcpp::SensorDataQoS(),
            std::bind(&LocalizationInitializer::imu_callback, this, std::placeholders::_1));

        clear_local_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
            "local_costmap/clear_entirely_local_costmap");
        clear_global_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
            "global_costmap/clear_entirely_global_costmap");
        ekf_set_pose_client_ = this->create_client<robot_localization::srv::SetPose>("set_pose");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        T_map_odom_msg_.header.frame_id = "map";
        T_map_odom_msg_.child_frame_id = "odom";
        T_map_odom_msg_.transform.rotation.w = 1.0;
        
        tf_publish_timer_ = this->create_wall_timer(
            66ms, std::bind(&LocalizationInitializer::publish_map_to_odom_tf, this));
        ekf_set_pose_response_timer_ = this->create_wall_timer(
            100ms, std::bind(&LocalizationInitializer::check_ekf_set_pose_response, this));

        RCLCPP_INFO(this->get_logger(), "Localization initializer started. Waiting for /initialpose...");
    }

private:
    static constexpr double kTransformTimeOffsetSeconds = 0.05;
    static constexpr auto kEkfSetPoseDelay = 250ms;
    static constexpr double kEkfSetPoseResponseTimeoutSeconds = 1.0;

    void publish_map_to_odom_tf()
    {
        T_map_odom_msg_.header.stamp =
            this->get_clock()->now() +
            rclcpp::Duration::from_seconds(kTransformTimeOffsetSeconds);
        tf_broadcaster_->sendTransform(T_map_odom_msg_);
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
    {
        imu_msg_ = msg;
        imu_received_ = true;
    }

    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Initial pose received, starting hard localization reset.");

        tf2::Quaternion q;

        if (imu_received_) {
            tf2::Quaternion imu_q;
            tf2::fromMsg(imu_msg_->orientation, imu_q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(imu_q).getRPY(roll, pitch, yaw);

            q.setRPY(0.0, 0.0, yaw);
            q.normalize();

            RCLCPP_INFO(this->get_logger(), "Using orientation from IMU sensor for initial pose.");
        } else {
            q = tf2::Quaternion(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
            q.normalize();

            RCLCPP_WARN(this->get_logger(), "IMU data not received. Using orientation from manually set initial pose.");
        }

        geometry_msgs::msg::PoseWithCovarianceStamped reset_pose = *msg;
        reset_pose.header.stamp = this->get_clock()->now();
        reset_pose.header.frame_id = "odom";
        reset_pose.pose.pose.position.z = 0.0;
        reset_pose.pose.pose.orientation = tf2::toMsg(q);

        // Hard reset mode: map and odom are the same frame after the reset.
        T_map_odom_msg_.transform.translation.x = 0.0;
        T_map_odom_msg_.transform.translation.y = 0.0;
        T_map_odom_msg_.transform.translation.z = 0.0;
        T_map_odom_msg_.transform.rotation.x = 0.0;
        T_map_odom_msg_.transform.rotation.y = 0.0;
        T_map_odom_msg_.transform.rotation.z = 0.0;
        T_map_odom_msg_.transform.rotation.w = 1.0;
        publish_map_to_odom_tf();

        reset_pose_pub_->publish(reset_pose);
        rclcpp::sleep_for(kEkfSetPoseDelay);
        set_ekf_pose(reset_pose);

        RCLCPP_INFO(this->get_logger(), "Hard localization reset requested with map -> odom identity.");
    }

    void set_ekf_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &reset_pose)
    {
        if (!ekf_set_pose_client_->service_is_ready() && !ekf_set_pose_client_->wait_for_service(250ms)) {
            RCLCPP_WARN(
                this->get_logger(),
                "EKF set_pose service is not available; EKF pose reset was not confirmed.");
            return;
        }

        auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        request->pose = reset_pose;
        ekf_set_pose_pending_ = true;
        ekf_set_pose_request_time_ = this->get_clock()->now();
        ekf_set_pose_future_ = ekf_set_pose_client_->async_send_request(request).future.share();
        RCLCPP_INFO(this->get_logger(), "Requested EKF pose reset via set_pose.");
    }

    void check_ekf_set_pose_response()
    {
        if (!ekf_set_pose_pending_)
        {
            return;
        }

        if (ekf_set_pose_future_.valid() &&
            ekf_set_pose_future_.wait_for(0s) == std::future_status::ready)
        {
            ekf_set_pose_future_.get();
            ekf_set_pose_pending_ = false;
            RCLCPP_INFO(this->get_logger(), "EKF set_pose service confirmed pose reset.");
            clear_costmaps();
            return;
        }

        const double elapsed = (this->get_clock()->now() - ekf_set_pose_request_time_).seconds();
        if (elapsed > kEkfSetPoseResponseTimeoutSeconds)
        {
            ekf_set_pose_pending_ = false;
            RCLCPP_WARN(
                this->get_logger(),
                "Timed out waiting for EKF set_pose service response after %.1f seconds; skipping costmap clear.",
                kEkfSetPoseResponseTimeoutSeconds);
        }
    }

    void clear_costmaps()
    {
        clear_costmap(clear_local_costmap_client_);
        clear_costmap(clear_global_costmap_client_);
    }

    void clear_costmap(const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr &client)
    {
        const auto service_name = client->get_service_name();

        if (!client->wait_for_service(1s)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Costmap clear service %s is not available; skipping clear for this initial pose update.",
                service_name);
            return;
        }

        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        client->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Requested costmap clear via %s.", service_name);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reset_pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_client_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_client_;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr ekf_set_pose_client_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_publish_timer_;
    rclcpp::TimerBase::SharedPtr ekf_set_pose_response_timer_;

    geometry_msgs::msg::TransformStamped T_map_odom_msg_;
    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedFuture ekf_set_pose_future_;
    rclcpp::Time ekf_set_pose_request_time_;
    bool ekf_set_pose_pending_ = false;
    bool imu_received_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationInitializer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
