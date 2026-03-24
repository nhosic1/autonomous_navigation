#include <algorithm>
#include <cmath>
#include <string>
#include <memory>
#include <nav2_util/node_utils.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include "autonomous_navigation/common/vehicle_constants.hpp"
#include "autonomous_navigation/nav2/nav2_pure_pursuit_controller.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;

namespace nav2_pure_pursuit_controller
{

    void Nav2PurePursuitController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent;

        auto node = node_.lock();

        costmap_ros_ = costmap_ros;
        tf_ = tf;
        plugin_name_ = name;
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        declare_parameter_if_not_declared(node, plugin_name_ + ".v_max", rclcpp::ParameterValue(1.0));
        declare_parameter_if_not_declared(node, plugin_name_ + ".ld_min", rclcpp::ParameterValue(1.5));
        declare_parameter_if_not_declared(node, plugin_name_ + ".K_v_turn", rclcpp::ParameterValue(1.0));
        declare_parameter_if_not_declared(node, plugin_name_ + ".K_v_stop", rclcpp::ParameterValue(-1.0));
        declare_parameter_if_not_declared(node, plugin_name_ + ".K_ld", rclcpp::ParameterValue(0.5));
        declare_parameter_if_not_declared(node, plugin_name_ + ".ld_goal", rclcpp::ParameterValue(0.75));

        double v_max;
        double ld_min;
        double K_v_turn;
        double K_v_stop;
        double K_ld;

        node->get_parameter(plugin_name_ + ".v_max", v_max);
        node->get_parameter(plugin_name_ + ".ld_min", ld_min);
        node->get_parameter(plugin_name_ + ".K_v_turn", K_v_turn);
        node->get_parameter(plugin_name_ + ".K_v_stop", K_v_stop);
        node->get_parameter(plugin_name_ + ".K_ld", K_ld);
        node->get_parameter(plugin_name_ + ".ld_goal", ld_goal_);

        pure_pursuit_controller_ = PurePursuitController(std::vector<Point>(), v_max, ld_min, K_v_turn, K_v_stop, K_ld);

        T_rear_axle_to_base_link_ = get_transform("base_link", "rear_axle");
    }

    void Nav2PurePursuitController::cleanup()
    {
        RCLCPP_INFO(logger_, "Cleaning up controller: %s of type nav2_pure_pursuit_controller::Nav2PurePursuitController", plugin_name_.c_str());
    }

    void Nav2PurePursuitController::activate()
    {
        RCLCPP_INFO(logger_, "Activating controller: %s of type nav2_pure_pursuit_controller::Nav2PurePursuitController", plugin_name_.c_str());
    }

    void Nav2PurePursuitController::deactivate()
    {
        RCLCPP_INFO(logger_, "Dectivating controller: %s of type nav2_pure_pursuit_controller::Nav2PurePursuitController", plugin_name_.c_str());
    }

    void Nav2PurePursuitController::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    geometry_msgs::msg::TwistStamped Nav2PurePursuitController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        // Get rear_axle pose in odom frame
        tf2::Transform pose_tf;
        tf2::fromMsg(pose.pose, pose_tf);
        tf2::Transform pose_rear_axle = pose_tf * T_rear_axle_to_base_link_;

        double x = pose_rear_axle.getOrigin().x();
        double y = pose_rear_axle.getOrigin().y();

        tf2::Quaternion Q_tf2 = pose_rear_axle.getRotation();
        double yaw = tf2::getYaw(Q_tf2);

        Pose current_pose(Point(x, y), yaw);
        const double current_linear_velocity = velocity.linear.x;

        double steering_angle, angular_velocity, linear_velocity;

        if (goal_checker != nullptr && has_goal_pose_ &&
            goal_checker->isGoalReached(pose.pose, goal_pose_odom_, velocity))
        {
            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = clock_->now();
            cmd_vel.header.frame_id = pose.header.frame_id;
            return cmd_vel;
        }

        try
        {
            std::tie(steering_angle, angular_velocity) = pure_pursuit_controller_.get_motion_controls(current_pose, current_linear_velocity);
        }
        catch (const std::runtime_error &e)
        {
            RCLCPP_ERROR(logger_, "Controller error: %s", e.what());
            throw nav2_core::ControllerException(e.what());
        }

        linear_velocity = angular_velocity * WHEEL_RADIUS;

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = clock_->now();
        cmd_vel.header.frame_id = pose.header.frame_id;

        cmd_vel.twist.linear.x = linear_velocity;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.linear.z = 0.0;

        cmd_vel.twist.angular.x = 0.0;
        cmd_vel.twist.angular.y = 0.0;
        cmd_vel.twist.angular.z = linear_velocity * std::tan(steering_angle) / WHEEL_BASE;


        return cmd_vel;
    }

    void Nav2PurePursuitController::setPlan(const nav_msgs::msg::Path &path)
    {
        global_plan_ = path;
        std::vector<Point> transformed_path;
        has_goal_pose_ = !path.poses.empty();

        T_map_to_odom_ = get_transform("odom", "map");

        // Transform the global plan from /map to /odom frame
        transformed_path.reserve(path.poses.size());
        for (const auto &path_point_msg : path.poses)
        {
            tf2::Transform path_point_tf;
            tf2::fromMsg(path_point_msg.pose, path_point_tf);
            tf2::Transform transformed_path_point = T_map_to_odom_ * path_point_tf;


            transformed_path.emplace_back(transformed_path_point.getOrigin().x(), transformed_path_point.getOrigin().y());
        }

        if (has_goal_pose_)
        {
            tf2::Transform goal_pose_map_tf;
            tf2::fromMsg(path.poses.back().pose, goal_pose_map_tf);
            tf2::Transform goal_pose_odom_tf = T_map_to_odom_ * goal_pose_map_tf;
            tf2::toMsg(goal_pose_odom_tf, goal_pose_odom_);

            if (ld_goal_ > 1e-6)
            {
                const double goal_yaw = tf2::getYaw(goal_pose_odom_tf.getRotation());
                const double goal_x = goal_pose_odom_tf.getOrigin().x();
                const double goal_y = goal_pose_odom_tf.getOrigin().y();

                transformed_path.emplace_back(
                    goal_x + ld_goal_ * std::cos(goal_yaw),
                    goal_y + ld_goal_ * std::sin(goal_yaw));
            }
        }

        pure_pursuit_controller_.set_path(transformed_path);


    }

    tf2::Transform Nav2PurePursuitController::get_transform(const std::string &target_frame, const std::string &source_frame)
    {
        geometry_msgs::msg::TransformStamped T_msg;
        tf2::Transform T;
        T.setIdentity();

        try
        {
            T_msg = tf_->lookupTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(logger_, "Could not transform %s to %s: %s", target_frame.c_str(), source_frame.c_str(), ex.what());
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

} // namespace nav2_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::Nav2PurePursuitController, nav2_core::Controller)
