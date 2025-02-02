#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "autonomous_navigation/pid_controller.hpp"
#include "autonomous_navigation/pure_pursuit.hpp"


#include "autonomous_navigation/vehicle_constants.hpp"

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

        odom_subscriber_.subscribe(this, "/autonomous_vehicle/odometry");
        joint_state_subscriber_.subscribe(this, "/autonomous_vehicle/joint_state");

        time_sync_ = std::make_shared<approximate_time_synchronizer>(approximate_time_policy(10), odom_subscriber_, joint_state_subscriber_);
        // time_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(0, 35000000)); // 0.035 sec
        time_sync_->registerCallback(std::bind(&PathFollower::motion_control_callback, this, std::placeholders::_1, std::placeholders::_2));
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
        // Get the current pose from odometry msg
        double x = odom_msg_ptr->pose.pose.position.x;
        double y = odom_msg_ptr->pose.pose.position.y;

        geometry_msgs::msg::Quaternion Q = odom_msg_ptr->pose.pose.orientation;

        tf2::Quaternion Q_tf2(Q.x, Q.y, Q.z, Q.w);
        double yaw = tf2::getYaw(Q_tf2);

        Pose current_pose(Point(x, y), yaw);

        // Get the current average angular velocity of the rear wheels from joint state msg
        double current_angular_velocity = (joint_state_msg_ptr->velocity[2] + joint_state_msg_ptr->velocity[3]) / 2;

        // Declare motion commands
        double steering_angle;
        double angular_velocity;
        
        std_msgs::msg::Float64 ang_vel_msg;
        std_msgs::msg::Float64 steering_ang_msg;

        try
        {
            std::tie(steering_angle, angular_velocity) = pure_pursuit_controller_.get_motion_controls(current_pose, current_angular_velocity);
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