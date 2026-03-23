#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include "autonomous_navigation/common/vehicle_constants.hpp"

class JointController : public rclcpp::Node
{
public:
    JointController() : Node("joint_controller")
    {
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, std::bind(&JointController::cmd_vel_callback, this, std::placeholders::_1));
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JointController::joint_control_callback, this, std::placeholders::_1));
        rear_left_wheel_ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/rear_left_wheel_joint/cmd_vel", 10);
        rear_right_wheel_ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/rear_right_wheel_joint/cmd_vel", 10);
        front_left_wheel_steering_ang_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/front_left_wheel_steering_joint/cmd_pos", 10);
        front_right_wheel_steering_ang_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/front_right_wheel_steering_joint/cmd_pos", 10);

        RCLCPP_INFO(this->get_logger(), "Joint controller is initialized. Listening to velocity commands...");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr &twist_msg_ptr)
    {
        target_linear_velocity_ = twist_msg_ptr->linear.x;
        target_angular_velocity_ = twist_msg_ptr->angular.z;
        cmd_vel_received_ = true;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Received velocity command: v=%.2f, omega=%.2f",
            target_linear_velocity_, target_angular_velocity_);
    }

    void joint_control_callback(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state_msg_ptr)
    {
        for (size_t i = 0; i < joint_state_msg_ptr->name.size(); ++i)
        {
            if (i < joint_state_msg_ptr->position.size())
                joint_positions_[joint_state_msg_ptr->name[i]] = joint_state_msg_ptr->position[i];
            if (i < joint_state_msg_ptr->velocity.size())
                joint_velocities_[joint_state_msg_ptr->name[i]] = joint_state_msg_ptr->velocity[i];
        }

        if (!cmd_vel_received_)
            return;

        double requested_v = target_linear_velocity_;
        double v = requested_v;
        double omega_vehicle = target_angular_velocity_;

        const double phi_max = atan(WHEEL_BASE / MINIMUM_TURNING_RADIUS);
        const double requested_phi =
            (std::abs(requested_v) < 1e-6) ? 0.0 : std::atan((omega_vehicle * WHEEL_BASE) / requested_v);
        double phi = std::clamp(requested_phi, -phi_max, phi_max);

        if (std::abs(phi - requested_phi) > 1e-6)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Requested steering angle %.2f rad exceeds the vehicle kinematic limit; clamping to %.2f rad",
                requested_phi, phi);
        }

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

            phi_L = std::atan(WHEEL_BASE / R_L_kingpin);
            phi_R = std::atan(WHEEL_BASE / R_R_kingpin);

            v_L = requested_v * (R_L_wheel / R);
            v_R = requested_v * (R_R_wheel / R);
        }

        bool joint_states_available =
            joint_positions_.count(front_left_steer_joint_) != 0 &&
            joint_positions_.count(front_right_steer_joint_) != 0 &&
            joint_velocities_.count(rear_left_wheel_joint_) != 0 &&
            joint_velocities_.count(rear_right_wheel_joint_) != 0;

        if (joint_states_available)
        {
            const double actual_phi_L = joint_positions_[front_left_steer_joint_];
            const double actual_phi_R = joint_positions_[front_right_steer_joint_];
            const double actual_v =
                (joint_velocities_[rear_left_wheel_joint_] + joint_velocities_[rear_right_wheel_joint_]) *
                0.5 * WHEEL_RADIUS;
            const double steering_error = std::max(std::abs(phi_L - actual_phi_L), std::abs(phi_R - actual_phi_R));
            const bool near_standstill = std::abs(actual_v) < standstill_velocity_threshold_;
            const bool sharp_turn_request = std::abs(phi) >= sharp_turn_angle_threshold_;
            const bool steering_not_ready = steering_error > steering_alignment_tolerance_;

            if (std::abs(requested_v) >= standstill_velocity_threshold_ &&
                near_standstill && sharp_turn_request && steering_not_ready)
            {
                v = 0.0;
                v_L = 0.0;
                v_R = 0.0;
                RCLCPP_DEBUG_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    3000,
                    "Holding linear velocity until steering aligns. actual_v=%.3f, steering_error=%.3f",
                    actual_v, steering_error);
            }
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

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Publishing controls: v=%.2f, w_L=%.2f, w_R=%.2f, phi_L=%.2f, phi_R=%.2f",
            v, omega_L, omega_R, phi_L, phi_R);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_wheel_ang_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_right_wheel_ang_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_wheel_steering_ang_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_wheel_steering_ang_publisher_;

    sensor_msgs::msg::JointState joint_state_;
    std::unordered_map<std::string, double> joint_positions_;
    std::unordered_map<std::string, double> joint_velocities_;
    double target_linear_velocity_{0.0};
    double target_angular_velocity_{0.0};
    bool cmd_vel_received_{false};
    const std::string front_left_steer_joint_{"front_left_wheel_steering_joint"};
    const std::string front_right_steer_joint_{"front_right_wheel_steering_joint"};
    const std::string rear_left_wheel_joint_{"rear_left_wheel_joint"};
    const std::string rear_right_wheel_joint_{"rear_right_wheel_joint"};
    const double standstill_velocity_threshold_{0.05};
    const double sharp_turn_angle_threshold_{0.2};
    const double steering_alignment_tolerance_{0.08};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointController>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
