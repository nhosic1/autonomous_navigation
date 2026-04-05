#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

class NavCmdGate : public rclcpp::Node
{
public:
    using Trigger = std_srvs::srv::Trigger;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    NavCmdGate()
        : Node("nav_cmd_gate")
    {
        nav_cmd_timeout_ = this->declare_parameter("nav_cmd_timeout", 0.3);

        nav_cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, std::bind(&NavCmdGate::nav_cmd_callback, this, std::placeholders::_1));
        teleop_cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_teleop", 10, std::bind(&NavCmdGate::teleop_cmd_callback, this, std::placeholders::_1));
        output_cmd_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        stop_nav_service_ = this->create_service<Trigger>(
            "/stop_nav", std::bind(&NavCmdGate::stop_nav_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));
        enable_nav_service_ = this->create_service<Trigger>(
            "/enable_nav", std::bind(&NavCmdGate::enable_nav_callback, this,
                                     std::placeholders::_1, std::placeholders::_2));

        navigate_to_pose_client_ =
            rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&NavCmdGate::watchdog_callback, this));
        RCLCPP_INFO(
            this->get_logger(),
            "nav_cmd_gate initialized. AUTO forwards /cmd_vel_nav to /cmd_vel. "
            "Any /cmd_vel_teleop message takes over until /enable_nav is called.");
    }

private:
    enum class ControlMode
    {
        AUTO,
        MANUAL
    };

    void nav_cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
    {
        if (control_mode_ != ControlMode::AUTO) {
            return;
        }

        last_nav_cmd_time_ = this->now();
        nav_cmd_timed_out_ = false;
        output_cmd_publisher_->publish(*msg);
    }

    void teleop_cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
    {
        if (control_mode_ != ControlMode::MANUAL) {
            control_mode_ = ControlMode::MANUAL;
            const auto cancel_summary = request_navigate_to_pose_cancel();
            RCLCPP_WARN(
                this->get_logger(),
                "Teleop control enabled. %s",
                cancel_summary.c_str());
        }

        output_cmd_publisher_->publish(*msg);
    }

    void stop_nav_callback(
        const std::shared_ptr<Trigger::Request> /*request*/,
        std::shared_ptr<Trigger::Response> response)
    {
        control_mode_ = ControlMode::MANUAL;

        geometry_msgs::msg::Twist zero_cmd;
        output_cmd_publisher_->publish(zero_cmd);
        
        const auto cancel_summary = request_navigate_to_pose_cancel();

        response->success = true;
        response->message =
            "Navigation commands blocked and zero velocity published. " + cancel_summary;

        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    }

    void enable_nav_callback(
        const std::shared_ptr<Trigger::Request> /*request*/,
        std::shared_ptr<Trigger::Response> response)
    {
        control_mode_ = ControlMode::AUTO;
        last_nav_cmd_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        nav_cmd_timed_out_ = false;
        response->success = true;
        response->message = "Navigation commands are enabled.";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void watchdog_callback()
    {
        if (control_mode_ != ControlMode::AUTO || nav_cmd_timeout_ <= 0.0 ||
            last_nav_cmd_time_.nanoseconds() == 0) {
            return;
        }

        const auto elapsed = (this->now() - last_nav_cmd_time_).seconds();
        if (elapsed > nav_cmd_timeout_ && !nav_cmd_timed_out_) {
            geometry_msgs::msg::Twist zero_cmd;
            output_cmd_publisher_->publish(zero_cmd);
            nav_cmd_timed_out_ = true;
            RCLCPP_WARN(
                this->get_logger(),
                "No /cmd_vel_nav received for %.3f s in AUTO mode. Publishing zero velocity.",
                elapsed);
        }
    }

    std::string request_navigate_to_pose_cancel()
    {
        constexpr auto action_name = "/navigate_to_pose";

        if (!navigate_to_pose_client_->action_server_is_ready()) {
            return std::string(action_name) + " server unavailable;";
        }

        (void)navigate_to_pose_client_->async_cancel_all_goals();
        return std::string(action_name) + " cancel requested;";
    }

    ControlMode control_mode_{ControlMode::AUTO};
    double nav_cmd_timeout_{0.3};
    bool nav_cmd_timed_out_{false};
    rclcpp::Time last_nav_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_cmd_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_cmd_publisher_;
    rclcpp::Service<Trigger>::SharedPtr stop_nav_service_;
    rclcpp::Service<Trigger>::SharedPtr enable_nav_service_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavCmdGate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
