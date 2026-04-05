#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <optional>
#include "autonomous_navigation/common/vehicle_constants.hpp"

class JointController : public rclcpp::Node
{
public:
    JointController() : Node("joint_controller")
    {
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&JointController::cmd_vel_callback, this, std::placeholders::_1));
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JointController::joint_control_callback, this, std::placeholders::_1));
        rear_left_wheel_ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/rear_left_wheel_joint/cmd_vel", 10);
        rear_right_wheel_ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/rear_right_wheel_joint/cmd_vel", 10);
        front_left_wheel_steering_ang_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/front_left_wheel_steering_joint/cmd_pos", 10);
        front_right_wheel_steering_ang_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/autonomous_vehicle/front_right_wheel_steering_joint/cmd_pos", 10);

        RCLCPP_INFO(this->get_logger(), "Joint controller is initialized. Listening to velocity commands...");
    }

private:
    struct JointFeedback
    {
        double steering_left;
        double steering_right;
        double linear_velocity;
    };

    std::pair<double, double> compute_wheel_linear_velocities(double v, double phi) const
    {
        if (std::abs(phi) < 1e-6)
            return std::make_pair(v, v);

        const double R = WHEEL_BASE / std::tan(phi);
        const double R_L_wheel = R - WHEEL_SEPARATION / 2.0;
        const double R_R_wheel = R + WHEEL_SEPARATION / 2.0;
        return std::make_pair(
            v * (R_L_wheel / R),
            v * (R_R_wheel / R));
    }

    std::pair<double, double> compute_wheel_angular_velocities(double v, double phi) const
    {
        const auto [v_L, v_R] = compute_wheel_linear_velocities(v, phi);
        return std::make_pair(v_L / WHEEL_RADIUS, v_R / WHEEL_RADIUS);
    }

    double apply_min_drive_velocity_limit(double v_requested, double phi) const
    {
        const auto [v_L, v_R] = compute_wheel_linear_velocities(v_requested, phi);
        double scale = 1.0;

        if (std::abs(v_L) > 1e-6 && std::abs(v_L) < v_min_drive_)
            scale = std::max(scale, v_min_drive_ / std::abs(v_L));

        if (std::abs(v_R) > 1e-6 && std::abs(v_R) < v_min_drive_)
            scale = std::max(scale, v_min_drive_ / std::abs(v_R));

        return v_requested * scale;
    }

    std::pair<double, double> compute_wheel_steering_angles(double phi) const
    {
        if (std::abs(phi) < 1e-6)
            return {phi, phi};

        const double R = WHEEL_BASE / std::tan(phi);
        const double R_L_kingpin = R - KINGPIN_WIDTH / 2.0;
        const double R_R_kingpin = R + KINGPIN_WIDTH / 2.0;
        return {
            std::atan(WHEEL_BASE / R_L_kingpin),
            std::atan(WHEEL_BASE / R_R_kingpin)};
    }

    double apply_turn_velocity_limit(double v_requested, double steering_error) const
    {
        const double abs_v_requested = std::abs(v_requested);
        if (abs_v_requested <= v_min_drive_ || steering_error <= steering_error_linear_drop_start_)
            return v_requested;

        if (steering_error >= steering_error_linear_drop_end_)
            return std::copysign(v_min_drive_, v_requested);

        // Define the line between:
        // (steering_error_linear_drop_start_, abs_v_requested) and
        // (steering_error_linear_drop_end_, v_min_drive_)
        const double slope =
            (v_min_drive_ - abs_v_requested) /
            (steering_error_linear_drop_end_ - steering_error_linear_drop_start_);
        const double intercept = abs_v_requested - slope * steering_error_linear_drop_start_;
        const double abs_limited_v = slope * steering_error + intercept;
        return std::copysign(abs_limited_v, v_requested);
    }

    std::optional<JointFeedback> read_joint_feedback(
        const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state_msg_ptr) const
    {
        std::optional<double> actual_phi_L;
        std::optional<double> actual_phi_R;
        std::optional<double> rear_left_wheel_omega;
        std::optional<double> rear_right_wheel_omega;

        for (size_t i = 0; i < joint_state_msg_ptr->name.size(); ++i)
        {
            if (i < joint_state_msg_ptr->position.size())
            {
                if (joint_state_msg_ptr->name[i] == front_left_steer_joint_)
                    actual_phi_L = joint_state_msg_ptr->position[i];
                else if (joint_state_msg_ptr->name[i] == front_right_steer_joint_)
                    actual_phi_R = joint_state_msg_ptr->position[i];
            }
            if (i < joint_state_msg_ptr->velocity.size())
            {
                if (joint_state_msg_ptr->name[i] == rear_left_wheel_joint_)
                    rear_left_wheel_omega = joint_state_msg_ptr->velocity[i];
                else if (joint_state_msg_ptr->name[i] == rear_right_wheel_joint_)
                    rear_right_wheel_omega = joint_state_msg_ptr->velocity[i];
            }
        }

        if (!actual_phi_L.has_value() || !actual_phi_R.has_value() ||
            !rear_left_wheel_omega.has_value() || !rear_right_wheel_omega.has_value())
        {
            return std::nullopt;
        }

        return JointFeedback{
            *actual_phi_L,
            *actual_phi_R,
            (*rear_left_wheel_omega + *rear_right_wheel_omega) * 0.5 * WHEEL_RADIUS};
    }

    double compute_limited_steering_angle(double v_requested, double omega_vehicle) const
    {
        const double phi_max = std::atan(WHEEL_BASE / MINIMUM_TURNING_RADIUS);
        const double requested_phi =
            (std::abs(v_requested) < 1e-6) ? 0.0 : std::atan((omega_vehicle * WHEEL_BASE) / v_requested);
        const double phi = std::clamp(requested_phi, -phi_max, phi_max);

        if (std::abs(phi - requested_phi) > 1e-6)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Requested steering angle %.2f rad exceeds the vehicle kinematic limit; clamping to %.2f rad",
                requested_phi, phi);
        }

        return phi;
    }

    void publish_commands(double phi_left, double phi_right, double omega_left, double omega_right) const
    {
        std_msgs::msg::Float64 left_steering_ang_msg, right_steering_ang_msg;
        std_msgs::msg::Float64 left_ang_vel_msg, right_ang_vel_msg;

        left_steering_ang_msg.data = phi_left;
        right_steering_ang_msg.data = phi_right;
        front_left_wheel_steering_ang_publisher_->publish(left_steering_ang_msg);
        front_right_wheel_steering_ang_publisher_->publish(right_steering_ang_msg);

        left_ang_vel_msg.data = omega_left;
        right_ang_vel_msg.data = omega_right;
        rear_left_wheel_ang_vel_publisher_->publish(left_ang_vel_msg);
        rear_right_wheel_ang_vel_publisher_->publish(right_ang_vel_msg);
    }

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
        if (!cmd_vel_received_)
            return;

        double v_requested = target_linear_velocity_;
        double v = v_requested;
        double omega_vehicle = target_angular_velocity_;
        const double phi = compute_limited_steering_angle(v_requested, omega_vehicle);

        const auto [phi_L, phi_R] = compute_wheel_steering_angles(phi);
        const auto joint_feedback = read_joint_feedback(joint_state_msg_ptr);

        if (joint_feedback.has_value())
        {
            const double steering_error = std::max(
                std::abs(phi_L - joint_feedback->steering_left),
                std::abs(phi_R - joint_feedback->steering_right));
            const bool near_standstill =
                std::abs(joint_feedback->linear_velocity) < standstill_velocity_threshold_;
            const bool sharp_turn_request = std::abs(phi) >= sharp_turn_angle_threshold_;
            const bool steering_not_ready = steering_error > steering_alignment_tolerance_;

            if (std::abs(v_requested) >= standstill_velocity_threshold_ &&
                near_standstill && sharp_turn_request && steering_not_ready)
            {
                v = 0.0;
                RCLCPP_DEBUG_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    3000,
                    "Holding linear velocity until steering aligns. actual_v=%.3f, steering_error=%.3f",
                    joint_feedback->linear_velocity, steering_error);
            }
            else if (sharp_turn_request && std::abs(v_requested) >= v_min_drive_ &&
                     steering_error > steering_error_linear_drop_start_)
            {
                v = apply_turn_velocity_limit(v_requested, steering_error);

                RCLCPP_DEBUG_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    3000,
                    "Reducing linear velocity during steering transition. requested_v=%.3f, limited_v=%.3f, steering_error=%.3f",
                    v_requested, v, steering_error);
            }
        }

        if (std::abs(v_requested) >= standstill_velocity_threshold_)
        {
            const double v_before_floor = v;
            v = apply_min_drive_velocity_limit(v, phi);

            if (std::abs(v - v_before_floor) > 1e-6)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    3000,
                    "Requested velocity %.2f m/s is below the minimum achievable value; raising to %.2f m/s",
                    v_before_floor, v);
            }
        }

        const auto [omega_L, omega_R] = compute_wheel_angular_velocities(v, phi);
        publish_commands(phi_L, phi_R, omega_L, omega_R);

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

    double target_linear_velocity_{0.0};
    double target_angular_velocity_{0.0};
    bool cmd_vel_received_{false};
    const std::string front_left_steer_joint_{"front_left_wheel_steering_joint"};
    const std::string front_right_steer_joint_{"front_right_wheel_steering_joint"};
    const std::string rear_left_wheel_joint_{"rear_left_wheel_joint"};
    const std::string rear_right_wheel_joint_{"rear_right_wheel_joint"};
    const double standstill_velocity_threshold_{0.05};
    const double v_min_drive_{0.25};
    const double sharp_turn_angle_threshold_{0.2};
    const double steering_alignment_tolerance_{0.08};
    const double steering_error_linear_drop_start_{0.04};
    const double steering_error_linear_drop_end_{0.14};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointController>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
