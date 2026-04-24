#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <stdexcept>
#include <cmath>
#include <limits>
#include "autonomous_navigation/common/vehicle_constants.hpp"

class WheelOdometryEstimator : public rclcpp::Node
{
public:
    WheelOdometryEstimator() : Node("wheel_odom_estimator"), last_time_(this->now()), x_(0.0), y_(0.0), theta_(0.0)
    {
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/autonomous_vehicle/odometry/wheel", 10);
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/autonomous_vehicle/twist/wheel", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        try
        {
            const tf2::Duration static_transform_wait = tf2::durationFromSec(3.0);

            const geometry_msgs::msg::TransformStamped T_base_link_rear_axle_msg =
                tf_buffer_->lookupTransform("base_link", "rear_axle", tf2::TimePointZero, static_transform_wait);
            tf2::fromMsg(T_base_link_rear_axle_msg.transform, T_base_link_rear_axle_);

            const geometry_msgs::msg::TransformStamped T_base_footprint_base_link_msg =
                tf_buffer_->lookupTransform("base_footprint", "base_link", tf2::TimePointZero, static_transform_wait);
            tf2::fromMsg(T_base_footprint_base_link_msg.transform, T_base_footprint_base_link_);
        }
        catch (const tf2::TransformException &ex)
        {
            throw std::runtime_error(std::string("wheel_odom_estimator startup failed: missing required static transform: ") + ex.what());
        }

        initialize_rear_axle_odometry_state();

        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&WheelOdometryEstimator::joint_state_callback, this, std::placeholders::_1));
    }

private:
    void initialize_rear_axle_odometry_state()
    {
        const tf2::Transform T_base_footprint_rear_axle = T_base_footprint_base_link_ * T_base_link_rear_axle_;
        x_ = T_base_footprint_rear_axle.getOrigin().x();
        y_ = T_base_footprint_rear_axle.getOrigin().y();
        z_ = T_base_footprint_rear_axle.getOrigin().z();
        double roll, pitch;
        T_base_footprint_rear_axle.getBasis().getRPY(roll, pitch, theta_);
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr &msg)
    {
        // Time since last update
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();

        if (dt < 0.03)
            return; // Skip processing

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
        const double v_L = velocities[rl_wheel] * WHEEL_RADIUS;
        const double v_R = velocities[rr_wheel] * WHEEL_RADIUS;
        const double v = (v_L + v_R) / 2.0;

        const double delta_L = positions[fl_steer];
        const double delta_R = positions[fr_steer];

        if (!std::isfinite(v_L) || !std::isfinite(v_R) ||
            !std::isfinite(delta_L) || !std::isfinite(delta_R))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Invalid wheel odometry joint data: v_L=%f, v_R=%f, delta_L=%f, delta_R=%f",
                v_L, v_R, delta_L, delta_R);
            return;
        }

        double R;

        // Compute turning radius of rear axle midpoint
        if (std::abs(delta_L) < 1e-6 && std::abs(delta_R) < 1e-6)
        {
            R = std::numeric_limits<double>::infinity();
        }
        else
        {
            const double R_L = WHEEL_BASE / std::tan(delta_L);
            const double R_R = WHEEL_BASE / std::tan(delta_R);
            R = (R_L + R_R) / 2.0;
        }

        // Angular velocity of rear axle midpoint
        const double omega = v / R;

        if (!std::isfinite(v) || !std::isfinite(omega))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Invalid wheel odometry motion estimate: v=%f, omega=%f",
                v, omega);
            return;
        }

        const double dtheta = omega * dt;
        
        // Integrate rear_axle pose in odom frame using the RK2 method.
        const double theta_mid = theta_ + 0.5 * dtheta;
        const double dx = v * std::cos(theta_mid) * dt;
        const double dy = v * std::sin(theta_mid) * dt;

        x_ += dx;
        y_ += dy;
        theta_ += dtheta;

        tf2::Quaternion q_odom_rear_axle;
        q_odom_rear_axle.setRPY(0.0, 0.0, theta_);
        q_odom_rear_axle.normalize();

        tf2::Transform T_odom_rear_axle;
        T_odom_rear_axle.setOrigin(tf2::Vector3(x_, y_, z_));
        T_odom_rear_axle.setRotation(q_odom_rear_axle);

        const tf2::Transform T_rear_axle_base_link = T_base_link_rear_axle_.inverse();
        const tf2::Transform T_odom_base_link = T_odom_rear_axle * T_rear_axle_base_link;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = T_odom_base_link.getOrigin().x();
        odom.pose.pose.position.y = T_odom_base_link.getOrigin().y();
        odom.pose.pose.position.z = T_odom_base_link.getOrigin().z();
        odom.pose.pose.orientation = tf2::toMsg(T_odom_base_link.getRotation());
        odom.pose.covariance = {
            0.2, 0, 0, 0, 0, 0,  // x
            0, 0.2, 0, 0, 0, 0,  // y
            0, 0, 9999, 0, 0, 0, // z
            0, 0, 0, 9999, 0, 0, // roll
            0, 0, 0, 0, 9999, 0, // pitch
            0, 0, 0, 0, 0, 0.3   // yaw
        };

        const tf2::Vector3 v_rear_axle_rear_axle(v, 0.0, 0.0);
        const tf2::Vector3 w_rear_axle_rear_axle(0.0, 0.0, omega);
        const tf2::Matrix3x3 R_base_link_rear_axle = T_base_link_rear_axle_.getBasis();
        const tf2::Vector3 p_base_link_rear_axle = T_base_link_rear_axle_.getOrigin();
        const tf2::Vector3 v_rear_axle_base_link = R_base_link_rear_axle * v_rear_axle_rear_axle;
        const tf2::Vector3 w_base_link_base_link = R_base_link_rear_axle * w_rear_axle_rear_axle;

        // Rigid-body velocity relation between two points:
        // v_rear = v_base + w x p_base->rear  =>  v_base = v_rear - w x p_base->rear
        const tf2::Vector3 v_base_link_base_link = v_rear_axle_base_link - w_base_link_base_link.cross(p_base_link_rear_axle);

        odom.twist.twist.linear.x = v_base_link_base_link.x();
        odom.twist.twist.linear.y = v_base_link_base_link.y();
        odom.twist.twist.linear.z = v_base_link_base_link.z();
        odom.twist.twist.angular.x = w_base_link_base_link.x();
        odom.twist.twist.angular.y = w_base_link_base_link.y();
        odom.twist.twist.angular.z = w_base_link_base_link.z();
        odom.twist.covariance = {
            0.2, 0, 0, 0, 0, 0,  // v_x
            0, 0.2, 0, 0, 0, 0,  // v_y
            0, 0, 9999, 0, 0, 0, // v_z
            0, 0, 0, 9999, 0, 0, // w_x
            0, 0, 0, 0, 9999, 0, // w_y
            0, 0, 0, 0, 0, 0.3   // w_z
        };

        odom_publisher_->publish(odom);

        geometry_msgs::msg::TwistWithCovarianceStamped twist;
        twist.header.stamp = now;
        twist.header.frame_id = odom.child_frame_id;
        twist.twist = odom.twist;
        twist_publisher_->publish(twist);

        last_time_ = now;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Time last_time_;
    tf2::Transform T_base_footprint_base_link_ = tf2::Transform::getIdentity();
    tf2::Transform T_base_link_rear_axle_ = tf2::Transform::getIdentity();
    double z_ = 0.0;
    double x_, y_, theta_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelOdometryEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
