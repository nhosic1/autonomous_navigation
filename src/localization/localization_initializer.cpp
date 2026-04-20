#include <memory>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "autonomous_navigation/common/vehicle_constants.hpp"

using namespace std::chrono_literals;

class LocalizationInitializer : public rclcpp::Node
{
public:
    LocalizationInitializer() : Node("localization_initializer_node")
    {
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&LocalizationInitializer::initial_pose_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autonomous_vehicle/imu_sensor/imu",
            rclcpp::SensorDataQoS(),
            std::bind(&LocalizationInitializer::imu_callback, this, std::placeholders::_1));

        set_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/set_pose", 10);

        reset_odom_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/reset_odom", 10);

        clear_local_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
            "local_costmap/clear_entirely_local_costmap");
        clear_global_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
            "global_costmap/clear_entirely_global_costmap");

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        initialize_map_to_odom_tf();
        tf_publish_timer_ = this->create_wall_timer(
            66ms, std::bind(&LocalizationInitializer::publish_map_to_odom_tf, this));

        RCLCPP_INFO(this->get_logger(), "Localization initializer started. Waiting for /initialpose...");
    }

private:
    static constexpr double kTransformTimeOffsetSeconds = 0.05;

    void initialize_map_to_odom_tf()
    {
        current_map_to_odom_tf_.header.frame_id = "map";
        current_map_to_odom_tf_.child_frame_id = "odom";
        current_map_to_odom_tf_.transform.translation.x = 0.0;
        current_map_to_odom_tf_.transform.translation.y = 0.0;
        current_map_to_odom_tf_.transform.translation.z = CHASSIS_HEIGHT;
        current_map_to_odom_tf_.transform.rotation.x = 0.0;
        current_map_to_odom_tf_.transform.rotation.y = 0.0;
        current_map_to_odom_tf_.transform.rotation.z = 0.0;
        current_map_to_odom_tf_.transform.rotation.w = 1.0;
    }

    void publish_map_to_odom_tf()
    {
        current_map_to_odom_tf_.header.stamp =
            this->get_clock()->now() +
            rclcpp::Duration::from_seconds(kTransformTimeOffsetSeconds);
        tf_broadcaster_->sendTransform(current_map_to_odom_tf_);
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
    {
        imu_msg_ = msg;
        imu_received_ = true;
    }

    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Initial pose received, resetting EKF and odometry.");

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

        current_map_to_odom_tf_.transform.translation.x = msg->pose.pose.position.x;
        current_map_to_odom_tf_.transform.translation.y = msg->pose.pose.position.y;
        current_map_to_odom_tf_.transform.translation.z = CHASSIS_HEIGHT;
        current_map_to_odom_tf_.transform.rotation = tf2::toMsg(q);

        // Publish the updated transform so the local odom frame is placed correctly in the map frame.
        publish_map_to_odom_tf();
        // rclcpp::sleep_for(200ms);

        // Reset wheel/visual odometry so they restart from a fresh local odom state.
        std_msgs::msg::Bool reset_odom_msg;
        reset_odom_msg.data = true;
        reset_odom_pub_->publish(reset_odom_msg);
        rclcpp::sleep_for(500ms);

        // Reset robot_localization to the identity pose in odom.
        geometry_msgs::msg::PoseWithCovarianceStamped set_pose_msg;
        set_pose_msg.header.stamp = this->get_clock()->now();
        set_pose_msg.header.frame_id = "odom";
        set_pose_msg.pose.pose.position.x = 0.0;
        set_pose_msg.pose.pose.position.y = 0.0;
        set_pose_msg.pose.pose.position.z = 0.0;
        set_pose_msg.pose.pose.orientation.w = 1.0;
        set_pose_msg.pose.covariance = {
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01};
        set_pose_pub_->publish(set_pose_msg);

        clear_costmap(clear_local_costmap_client_);
        clear_costmap(clear_global_costmap_client_);

        RCLCPP_INFO(this->get_logger(), "Updated dynamic transform: map -> odom");
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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_odom_pub_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_client_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_client_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_publish_timer_;

    geometry_msgs::msg::TransformStamped current_map_to_odom_tf_;
    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_;
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
