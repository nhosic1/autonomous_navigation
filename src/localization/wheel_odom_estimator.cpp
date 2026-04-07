#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>
#include "autonomous_navigation/common/vehicle_constants.hpp"

class WheelOdometryEstimator : public rclcpp::Node
{
public:
    WheelOdometryEstimator() : Node("wheel_odom_estimator"), last_time_(this->now()), x_(0.0), y_(0.0), theta_(0.0), reset_odom_(false)
    {
        std::cout << "Initializing!" << std::endl;
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/autonomous_vehicle/odometry/wheel", 10);
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&WheelOdometryEstimator::joint_state_callback, this, std::placeholders::_1));

        reset_odom_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "reset_odom", 1,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                if (msg->data)
                {
                    reset_odom_ = true;
                    x_ = 0.0;
                    y_ = 0.0;
                    theta_ = 0.0;
                    last_time_ = this->now();

                    RCLCPP_INFO(this->get_logger(), "Wheel odometry is reset.");
                }
            });
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr &msg)
    {
        // Time since last update
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();

        if (dt < 0.03)
            return;  // Skip processing
            
        std::unordered_map<std::string, double> positions, velocities;
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (i < msg->position.size())
            positions[msg->name[i]] = msg->position[i];
            if (i < msg->velocity.size())
            velocities[msg->name[i]] = msg->velocity[i];
        }

        // Required joints for steering and driving
        const std::string fl_steer = "front_left_wheel_steering_joint";
        const std::string fr_steer = "front_right_wheel_steering_joint";
        const std::string rl_wheel = "rear_left_wheel_joint";
        const std::string rr_wheel = "rear_right_wheel_joint";

        // Ensure all required joints are available
        if (positions.count(fl_steer) == 0 || positions.count(fr_steer) == 0 ||
            velocities.count(rl_wheel) == 0 || velocities.count(rr_wheel) == 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Missing joint data");
            return;
        }

        // Convert rear wheel angular velocities to linear velocity of rear axle midpoint
        double v_L = velocities[rl_wheel] * WHEEL_RADIUS;
        double v_R = velocities[rr_wheel] * WHEEL_RADIUS;
        double v = (v_L + v_R) / 2.0;

        std::cout << "Received joint state: " << std::endl;
        std::cout << "v_L: " << v_L << ", v_R: " << v_R << ", v: " << v << std::endl;

        const double delta_L = positions[fl_steer];
        const double delta_R = positions[fr_steer];

        double R;

        // Compute turning radius of rear axle midpoint
        if (std::abs(delta_L) < 1e-6 && std::abs(delta_R) < 1e-6)
        {
            R = std::numeric_limits<double>::infinity();
        }
        else
        {
            double R_L = WHEEL_BASE / std::tan(delta_L);
            double R_R = WHEEL_BASE / std::tan(delta_R);
            R = (R_L + R_R) / 2.0;

            std::cout << "delta_L: " << delta_L << ", delta_R: " << delta_R << std::endl;
            std::cout << "R_L: " << R_L << ", R_R: " << R_R << ", R: " << R << std::endl;
        }

        // Angular velocity of rear axle midpoint
        double omega = v / R;

        std::cout << "w_L: " << velocities[rl_wheel] << ", w_R: " << velocities[rr_wheel] << ", w: " << omega << std::endl;
        
        // Integrate pose using velocity and orientation
        double dx = v * std::cos(theta_) * dt;
        double dy = v * std::sin(theta_) * dt;
        double dtheta = omega * dt;

        x_ += dx;
        y_ += dy;
        theta_ += dtheta;

        // Fill and publish odometry message
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "rear_axle";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.pose.covariance = {
            0.2, 0, 0, 0, 0, 0, // x
            0, 0.2, 0, 0, 0, 0,  // y
            0, 0, 9999, 0, 0, 0, // z
            0, 0, 0, 9999, 0, 0, // roll
            0, 0, 0, 0, 9999, 0, // pitch
            0, 0, 0, 0, 0, 0.3  // yaw
        };

        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = omega;
        odom.twist.covariance = {
            0.2, 0, 0, 0, 0, 0, // v_x
            0, 0.2, 0, 0, 0, 0,  // v_y
            0, 0, 9999, 0, 0, 0, // v_z
            0, 0, 0, 9999, 0, 0, // w_x
            0, 0, 0, 0, 9999, 0, // w_y
            0, 0, 0, 0, 0, 0.3  // w_z
        };

        odom_publisher_->publish(odom);

        last_time_ = now;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_odom_subscription_;

    rclcpp::Time last_time_;
    double x_, y_, theta_;
    bool reset_odom_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelOdometryEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
