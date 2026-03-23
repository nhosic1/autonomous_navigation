#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "autonomous_navigation/common/vehicle_constants.hpp"
#include "autonomous_navigation/control/pid_controller.hpp"
#include "autonomous_navigation/control/pure_pursuit.hpp"

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::JointState> approximate_time_policy;
typedef message_filters::Synchronizer<approximate_time_policy> approximate_time_synchronizer;

class PathFollower : public rclcpp::Node
{
public:
    PathFollower() : Node("path_follower")
    {
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>("/autonomous_vehicle/path", 10, std::bind(&PathFollower::path_callback, this, std::placeholders::_1));
        rear_left_wheel_ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/rear_left_wheel_joint/cmd_vel", 10);
        rear_right_wheel_ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/rear_right_wheel_joint/cmd_vel", 10);
        front_left_wheel_steering_ang_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/front_left_wheel_steering_joint/cmd_pos", 10);
        front_right_wheel_steering_ang_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/front_right_wheel_steering_joint/cmd_pos", 10);

        odom_subscriber_.subscribe(this, "/odometry/filtered");
        joint_state_subscriber_.subscribe(this, "/autonomous_vehicle/joint_state");

        time_sync_ = std::make_shared<approximate_time_synchronizer>(approximate_time_policy(10), odom_subscriber_, joint_state_subscriber_);
        time_sync_->registerCallback(std::bind(&PathFollower::motion_control_callback, this, std::placeholders::_1, std::placeholders::_2));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        T_base_to_rear_axle_ = get_transform("base_link", "rear_axle");
    }

private:
    void path_callback(const nav_msgs::msg::Path::ConstSharedPtr &path_msg_ptr)
    {
        path_.clear();

        path_.reserve(path_msg_ptr->poses.size());
        for (const auto &pose : path_msg_ptr->poses)
            path_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    void motion_control_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg_ptr, const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state_msg_ptr)
    {
        tf2::Transform pose_base;
        tf2::fromMsg(odom_msg_ptr->pose.pose, pose_base);
        tf2::Transform pose_rear_axle = pose_base * T_base_to_rear_axle_;

        // Get the current pose from odometry msg
        double x = pose_rear_axle.getOrigin().x();
        double y = pose_rear_axle.getOrigin().y();

        tf2::Quaternion Q_tf2 = pose_rear_axle.getRotation();
        double yaw = tf2::getYaw(Q_tf2);

        Pose current_pose(Point(x, y), yaw);

        // Estimate the current linear speed from the average rear wheel angular velocity.
        const double current_linear_velocity =
            ((joint_state_msg_ptr->velocity[2] + joint_state_msg_ptr->velocity[3]) / 2.0) * WHEEL_RADIUS;

        // Declare motion commands
        double steering_angle;
        double angular_velocity;
        
        std_msgs::msg::Float64 ang_vel_msg;
        std_msgs::msg::Float64 steering_ang_msg;

        try
        {
            std::tie(steering_angle, angular_velocity) = pure_pursuit_controller_.get_motion_controls(current_pose, current_linear_velocity);
        }
        catch (const std::runtime_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());

            // Stop the vehicle
            ang_vel_msg.data = 0.0;
            rear_left_wheel_ang_vel_publisher_->publish(ang_vel_msg);
            rear_right_wheel_ang_vel_publisher_->publish(ang_vel_msg);

            rclcpp::shutdown();
            return;
        }

        // Publish control commands
        ang_vel_msg.data = angular_velocity;
        steering_ang_msg.data = steering_angle;

        rear_left_wheel_ang_vel_publisher_->publish(ang_vel_msg);
        rear_right_wheel_ang_vel_publisher_->publish(ang_vel_msg);
        front_left_wheel_steering_ang_publisher_->publish(steering_ang_msg);
        front_right_wheel_steering_ang_publisher_->publish(steering_ang_msg);
    }

    tf2::Transform get_transform(const std::string &target_frame, const std::string &source_frame)
    {
        geometry_msgs::msg::TransformStamped T_msg;
        tf2::Transform T;
        T.setIdentity();

        try
        {
            T_msg = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), source_frame.c_str(), ex.what());
            return T;
        }

        // Convert translation
        tf2::Vector3 t(
            T_msg.transform.translation.x,
            T_msg.transform.translation.y,
            T_msg.transform.translation.z);

        // Convert rotation (quaternion)
        tf2::Quaternion q(
            T_msg.transform.rotation.x,
            T_msg.transform.rotation.y,
            T_msg.transform.rotation.z,
            T_msg.transform.rotation.w);

        // Set translation and rotation to the tf2::Transform object
        T.setOrigin(t);
        T.setRotation(q);

        return T;
    }

    std::vector<Point> generate_figure_eight(double a = 11.0, double b = 11.0, double num_points = 100)
    {
        std::vector<Point> points;
        
        for (int i = 0; i < num_points; ++i)
        {
            double t = (2 * M_PI * i) / num_points; 
            double x = a * sin(t) + 1.5;                 
            double y = b * sin(t) * cos(t);           
            points.push_back(Point(x, y));
        }
        
        return points;
    }

    std::shared_ptr<approximate_time_synchronizer> time_sync_;

    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> joint_state_subscriber_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_wheel_ang_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_right_wheel_ang_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_wheel_steering_ang_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_wheel_steering_ang_publisher_;

    std::vector<Point> path_ = generate_figure_eight();
    PurePursuitController pure_pursuit_controller_ = PurePursuitController(path_, 0.5, 1.9, 1.0);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    tf2::Transform T_base_to_rear_axle_;

    double current_steering_angle_;
    double current_linear_velocity_;

    sensor_msgs::msg::JointState joint_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollower>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
