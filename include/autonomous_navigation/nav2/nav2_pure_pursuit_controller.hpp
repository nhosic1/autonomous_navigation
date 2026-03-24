#ifndef AUTONOMOUS_NAVIGATION__NAV2_PURE_PURSUIT_CONTROLLER_HPP_
#define AUTONOMOUS_NAVIGATION__NAV2_PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include <nav2_core/controller.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "autonomous_navigation/control/pure_pursuit.hpp"

namespace nav2_pure_pursuit_controller
{

class Nav2PurePursuitController : public nav2_core::Controller
{
public:
  Nav2PurePursuitController() = default;
  ~Nav2PurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  tf2::Transform get_transform(const std::string &target_frame, const std::string &source_frame);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("Nav2PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  tf2::Transform T_rear_axle_to_base_link_, T_map_to_odom_;

  nav_msgs::msg::Path global_plan_;
  PurePursuitController pure_pursuit_controller_;
  geometry_msgs::msg::Pose goal_pose_odom_;
  double ld_goal_{0.75};
  bool has_goal_pose_{false};
};

}  // namespace nav2_pure_pursuit_controller

#endif  // AUTONOMOUS_NAVIGATION__NAV2_PURE_PURSUIT_CONTROLLER_HPP_
