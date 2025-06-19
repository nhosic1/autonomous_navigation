#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "autonomous_navigation/vehicle_constants.hpp"

class JointController : public rclcpp::Node
{
public:
    JointController() : Node("joint_controller")
    {
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&JointController::joint_control_callback, this, std::placeholders::_1));
        rear_left_wheel_ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/rear_left_wheel_joint/cmd_vel", 10);
        rear_right_wheel_ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/rear_right_wheel_joint/cmd_vel", 10);
        front_left_wheel_steering_ang_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/front_left_wheel_steering_joint/cmd_pos", 10);
        front_right_wheel_steering_ang_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/front_right_wheel_steering_joint/cmd_pos", 10);

        RCLCPP_INFO(this->get_logger(), "Joint controller is initialized. Listening to velocity commands...");
    }

private:
    void joint_control_callback(const geometry_msgs::msg::Twist::ConstSharedPtr &twist_msg_ptr)
    {
        double v = twist_msg_ptr->linear.x;
        double omega_vehicle = twist_msg_ptr->angular.z;

        RCLCPP_INFO(this->get_logger(), "Received velocity command: v=%.2f, omega=%.2f", v, omega_vehicle);

        double phi = (std::abs(v) < 1e-6) ? 0.0 : std::atan((omega_vehicle * WHEEL_BASE) / v);

        // Define steering limit of bicycle model
        double steering_limit = atan(WHEEL_BASE / (MINIMUM_TURNING_RADIUS + WHEEL_SEPARATION / 2));
        phi = std::clamp(phi, -steering_limit, steering_limit);

        RCLCPP_INFO(this->get_logger(), "Calculated steering angle (bicycle model): phi=%.2f", phi);

        double phi_L, phi_R;
        double v_L, v_R;

        if (std::abs(phi) < 1e-6)
        {
            phi_L = phi_R = phi;
            v_L = v_R = v;
        }
        else
        {
            double R = WHEEL_BASE / std::tan(phi);
            double R_L_kingpin = R - KINGPIN_WIDTH / 2.0;
            double R_R_kingpin = R + KINGPIN_WIDTH / 2.0;
            double R_L_wheel = R - WHEEL_SEPARATION / 2.0;
            double R_R_wheel = R + WHEEL_SEPARATION / 2.0;

            RCLCPP_INFO(this->get_logger(), "Calculated radii: R=%.2f, R_L_kingpin=%.2f, R_R_kingpin=%.2f", R, R_L_kingpin, R_R_kingpin);

            phi_L = std::atan(WHEEL_BASE / R_L_kingpin);
            phi_R = std::atan(WHEEL_BASE / R_R_kingpin);

            v_L = v * (R_L_wheel / R);
            v_R = v * (R_R_wheel / R);
        }

        double omega_L = v_L / WHEEL_RADIUS;
        double omega_R = v_R / WHEEL_RADIUS;

        std_msgs::msg::Float64 left_steering_ang_msg, right_steering_ang_msg;
        std_msgs::msg::Float64 left_ang_vel_msg, right_ang_vel_msg;

        // Publish control commands
        left_steering_ang_msg.data = phi_L;
        right_steering_ang_msg.data = phi_R;
        front_left_wheel_steering_ang_publisher_->publish(left_steering_ang_msg);
        front_right_wheel_steering_ang_publisher_->publish(right_steering_ang_msg);

        left_ang_vel_msg.data = omega_L;
        right_ang_vel_msg.data = omega_R;
        rear_left_wheel_ang_vel_publisher_->publish(left_ang_vel_msg);
        rear_right_wheel_ang_vel_publisher_->publish(right_ang_vel_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing controls: v=%.2f, w_L=%.2f, w_R=%.2f, phi_L=%.2f, phi_R=%.2f", v, omega_L, omega_R, phi_L, phi_R);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_wheel_ang_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_right_wheel_ang_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_wheel_steering_ang_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_wheel_steering_ang_publisher_;

    sensor_msgs::msg::JointState joint_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointController>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}