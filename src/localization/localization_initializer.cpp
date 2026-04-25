#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
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

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autonomous_vehicle/imu_sensor/imu",
            rclcpp::SensorDataQoS(),
            std::bind(&LocalizationInitializer::imu_callback, this, std::placeholders::_1));

        clear_local_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
            "local_costmap/clear_entirely_local_costmap");
        clear_global_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
            "global_costmap/clear_entirely_global_costmap");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        T_map_odom_msg_.header.frame_id = "map";
        T_map_odom_msg_.child_frame_id = "odom";
        T_map_odom_msg_.transform.rotation.w = 1.0;
        
        tf_publish_timer_ = this->create_wall_timer(
            66ms, std::bind(&LocalizationInitializer::publish_map_to_odom_tf, this));

        RCLCPP_INFO(this->get_logger(), "Localization initializer started. Waiting for /initialpose...");
    }

private:
    static constexpr double kTransformTimeOffsetSeconds = 0.05;

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
        RCLCPP_INFO(this->get_logger(), "Initial pose received, updating map -> odom transform.");

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

        geometry_msgs::msg::TransformStamped T_odom_base_footprint_msg;
        try {
            T_odom_base_footprint_msg = tf_buffer_->lookupTransform(
                "odom", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.5));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to update initial pose because odom -> base_footprint is unavailable: %s",
                ex.what());
            return;
        }

        tf2::Transform T_odom_base_footprint;
        tf2::fromMsg(T_odom_base_footprint_msg.transform, T_odom_base_footprint);

        tf2::Transform T_map_base_footprint_target;
        T_map_base_footprint_target.setOrigin(tf2::Vector3(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z));
        T_map_base_footprint_target.setRotation(q);

        const tf2::Transform T_map_odom_target =
            T_map_base_footprint_target * T_odom_base_footprint.inverse();
        T_map_odom_msg_.transform = tf2::toMsg(T_map_odom_target);

        // Publish the updated transform so the local odom frame is placed correctly in the map frame.
        publish_map_to_odom_tf();

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
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_client_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_client_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_publish_timer_;

    geometry_msgs::msg::TransformStamped T_map_odom_msg_;
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
