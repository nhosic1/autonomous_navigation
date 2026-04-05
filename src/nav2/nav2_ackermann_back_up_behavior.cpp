#include "autonomous_navigation/nav2/nav2_ackermann_back_up_behavior.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_util/robot_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

namespace autonomous_navigation
{

namespace
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace

AckermannBackUp::AckermannBackUp()
: nav2_behaviors::TimedBehavior<ActionT>(),
  feedback_(std::make_shared<ActionT::Feedback>())
{
}

AckermannBackUp::ResultStatus
AckermannBackUp::onRun(const std::shared_ptr<const ActionT::Goal> command)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!getCurrentPose(current_pose)) {
    RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
    return ResultStatus{Status::FAILED, ActionT::Goal::TF_ERROR};
  }

  command_time_allowance_ = command->time_allowance;
  end_time_ = this->steady_clock_.now() + command_time_allowance_;
  const auto selected_side = selectReverseSide(current_pose, command->goal_pose);
  if (!selected_side.has_value()) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Ackermann recovery could not find a feasible reverse side from the start pose.");
    return ResultStatus{Status::FAILED, ActionT::Goal::COLLISION_AHEAD};
  }

  selected_side_ = *selected_side;
  phase_start_pose_ = current_pose;
  RCLCPP_INFO(this->logger_, "Ackermann recovery entering reverse phase.");
  RCLCPP_INFO(
    this->logger_,
    "Starting Ackermann recovery with %s reverse at %.2f m/s.",
    sideName(selected_side_).c_str(), reverse_linear_vel_);

  return ResultStatus{Status::SUCCEEDED, ActionT::Goal::NONE};
}

AckermannBackUp::ResultStatus AckermannBackUp::onCycleUpdate()
{
  const rclcpp::Time now = this->steady_clock_.now();
  const rclcpp::Duration time_remaining = end_time_ - now;

  geometry_msgs::msg::PoseStamped current_pose;
  if (!getCurrentPose(current_pose)) {
    this->stopRobot();
    return ResultStatus{Status::FAILED, ActionT::Goal::TF_ERROR};
  }

  const double heading_delta = yawDeltaFromPhaseStart(current_pose);
  const double heading_delta_deg = heading_delta * 180.0 / M_PI;
  const double abs_heading_delta = std::abs(heading_delta);
  const double time_remaining_sec = time_remaining.seconds();
  feedback_->heading_change = static_cast<float>(heading_delta);
  this->action_server_->publish_feedback(feedback_);

  RCLCPP_INFO_THROTTLE(
    this->logger_,
    *this->node_.lock()->get_clock(),
    1000,
    "Ackermann recovery progress: heading_change=%.1f deg (min %.1f, max %.1f), time_remaining=%.2f s.",
    heading_delta_deg,
    min_reverse_heading_change_rad_ * 180.0 / M_PI,
    max_reverse_heading_change_rad_ * 180.0 / M_PI,
    time_remaining_sec);

  if (abs_heading_delta >= min_reverse_heading_change_rad_) {
    const bool forward_escape_feasible = isForwardEscapeFeasible(current_pose);
    if (forward_escape_feasible) {
      this->stopRobot();
      RCLCPP_INFO(
        this->logger_,
        "Ackermann recovery succeeded after %.1f deg yaw change; forward escape path is feasible.",
        heading_delta_deg);
      return ResultStatus{Status::SUCCEEDED, ActionT::Goal::NONE};
    }

    RCLCPP_INFO_THROTTLE(
      this->logger_,
      *this->node_.lock()->get_clock(),
      1000,
      "Ackermann recovery reached %.1f deg yaw change but counter-steer forward probe is still blocked.",
      heading_delta_deg);
  }

  if (time_remaining_sec < 0.0 && command_time_allowance_.seconds() > 0.0) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Ackermann recovery timed out after reaching %.1f deg yaw change (min %.1f deg, max %.1f deg).",
      heading_delta_deg,
      min_reverse_heading_change_rad_ * 180.0 / M_PI,
      max_reverse_heading_change_rad_ * 180.0 / M_PI);
    return ResultStatus{Status::FAILED, ActionT::Goal::TIMEOUT};
  }

  if (abs_heading_delta >= max_reverse_heading_change_rad_) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Ackermann recovery reached its reverse heading limit at %.1f deg without finding a forward escape.",
      heading_delta_deg);
    return ResultStatus{Status::FAILED, ActionT::Goal::COLLISION_AHEAD};
  }

  const geometry_msgs::msg::Twist cmd_vel = buildReverseCommand(selected_side_);
  if (!isCommandCollisionFree(current_pose, cmd_vel, simulate_ahead_time_)) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Ackermann recovery %s reverse would collide within the receding horizon after %.1f deg yaw change.",
      sideName(selected_side_).c_str(),
      heading_delta_deg);
    return ResultStatus{Status::FAILED, ActionT::Goal::COLLISION_AHEAD};
  }

  this->vel_pub_->publish(cmd_vel);
  return ResultStatus{Status::RUNNING, ActionT::Goal::NONE};
}

nav2_core::CostmapInfoType AckermannBackUp::getResourceInfo()
{
  return CostmapInfoType::LOCAL;
}

void AckermannBackUp::onConfigure()
{
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock behavior server node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".simulate_ahead_time", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".forward_probe_time", rclcpp::ParameterValue(6.0));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".sample_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".reverse_linear_vel", rclcpp::ParameterValue(-0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".reverse_angular_vel", rclcpp::ParameterValue(0.175));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".forward_linear_vel", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".forward_angular_vel", rclcpp::ParameterValue(0.175));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".max_reverse_heading_change_deg", rclcpp::ParameterValue(90.0));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".min_reverse_heading_change_deg", rclcpp::ParameterValue(25.0));

  node->get_parameter(behavior_name_ + ".simulate_ahead_time", simulate_ahead_time_);
  node->get_parameter(behavior_name_ + ".forward_probe_time", forward_probe_time_);
  node->get_parameter(behavior_name_ + ".sample_time_step", sample_time_step_);
  node->get_parameter(behavior_name_ + ".reverse_linear_vel", reverse_linear_vel_);
  node->get_parameter(behavior_name_ + ".reverse_angular_vel", reverse_angular_vel_);
  node->get_parameter(behavior_name_ + ".forward_linear_vel", forward_linear_vel_);
  node->get_parameter(behavior_name_ + ".forward_angular_vel", forward_angular_vel_);
  double max_reverse_heading_change_deg{};
  double min_reverse_heading_change_deg{};
  node->get_parameter(behavior_name_ + ".max_reverse_heading_change_deg", max_reverse_heading_change_deg);
  node->get_parameter(behavior_name_ + ".min_reverse_heading_change_deg", min_reverse_heading_change_deg);

  max_reverse_heading_change_rad_ =
    std::max(0.0, max_reverse_heading_change_deg) * M_PI / 180.0;
  min_reverse_heading_change_rad_ =
    std::clamp(min_reverse_heading_change_deg, 0.0, max_reverse_heading_change_deg) *
    M_PI / 180.0;
  reverse_linear_vel_ = -std::abs(reverse_linear_vel_);
  reverse_angular_vel_ = std::abs(reverse_angular_vel_);
  forward_linear_vel_ = std::abs(forward_linear_vel_);
  forward_angular_vel_ = std::abs(forward_angular_vel_);
  forward_probe_time_ = std::max(0.1, forward_probe_time_);
  sample_time_step_ = std::max(0.02, sample_time_step_);

  RCLCPP_INFO(
    this->logger_,
    "Configured AckermannBackUp: reverse_linear_vel=%.2f m/s, reverse_angular_vel=%.2f rad/s, "
    "max_reverse_heading_change=%.1f deg.",
    reverse_linear_vel_, reverse_angular_vel_, max_reverse_heading_change_deg);
}

bool AckermannBackUp::getCurrentPose(geometry_msgs::msg::PoseStamped & pose) const
{
  return nav2_util::getCurrentPose(
    pose, *this->tf_, this->local_frame_, this->robot_base_frame_, this->transform_tolerance_);
}

bool AckermannBackUp::isGoalPoseValid(const geometry_msgs::msg::PoseStamped & goal_pose) const
{
  return !goal_pose.header.frame_id.empty();
}

std::optional<AckermannBackUp::TurnSide> AckermannBackUp::choosePreferredReverseSide(
  const geometry_msgs::msg::PoseStamped & goal_pose) const
{
  geometry_msgs::msg::PoseStamped goal_pose_for_tf = goal_pose;
  goal_pose_for_tf.header.stamp = builtin_interfaces::msg::Time{};

  geometry_msgs::msg::PoseStamped goal_pose_base;
  if (!nav2_util::transformPoseInTargetFrame(
      goal_pose_for_tf, goal_pose_base, *this->tf_, this->robot_base_frame_, this->transform_tolerance_))
  {
    return std::nullopt;
  }

  const double goal_lateral_offset = goal_pose_base.pose.position.y;
  if (std::abs(goal_lateral_offset) < 1e-3) {
    return TurnSide::LEFT;
  }

  return goal_lateral_offset > 0.0 ? TurnSide::RIGHT : TurnSide::LEFT;
}

double AckermannBackUp::yawDeltaFromPhaseStart(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  const double start_yaw = tf2::getYaw(phase_start_pose_.pose.orientation);
  const double current_yaw = tf2::getYaw(pose.pose.orientation);
  return normalizeAngle(current_yaw - start_yaw);
}

bool AckermannBackUp::isCommandCollisionFree(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist & cmd_vel,
  double simulation_time) const
{
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  const double max_sim_time = std::max(0.0, simulation_time);

  bool fetch_data = true;
  double simulated_time = 0.0;
  while (simulated_time < max_sim_time) {
    const double dt = std::min(sample_time_step_, max_sim_time - simulated_time);
    simulated_time += dt;

    const double yaw = pose2d.theta;
    pose2d.x += cmd_vel.linear.x * std::cos(yaw) * dt;
    pose2d.y += cmd_vel.linear.x * std::sin(yaw) * dt;
    pose2d.theta = normalizeAngle(pose2d.theta + cmd_vel.angular.z * dt);

    if (!this->local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }

  return true;
}

bool AckermannBackUp::isForwardEscapeFeasible(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  geometry_msgs::msg::Twist forward_probe_cmd;
  const double reverse_turn_sign = selected_side_ == TurnSide::LEFT ? -1.0 : 1.0;
  forward_probe_cmd.linear.x = forward_linear_vel_;
  forward_probe_cmd.angular.z = reverse_turn_sign * forward_angular_vel_;
  return isCommandCollisionFree(current_pose, forward_probe_cmd, forward_probe_time_);
}

bool AckermannBackUp::isReverseSideFeasible(
  const geometry_msgs::msg::PoseStamped & current_pose, TurnSide side) const
{
  const geometry_msgs::msg::Twist cmd_vel = buildReverseCommand(side);
  const double required_heading_change =
    std::max(min_reverse_heading_change_rad_, sample_time_step_ * reverse_angular_vel_);
  const double simulation_time =
    required_heading_change / std::max(1e-3, reverse_angular_vel_);
  return isCommandCollisionFree(current_pose, cmd_vel, simulation_time);
}

std::optional<AckermannBackUp::TurnSide> AckermannBackUp::selectReverseSide(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose) const
{
  TurnSide preferred_side = TurnSide::LEFT;
  if (isGoalPoseValid(goal_pose)) {
    const auto goal_preferred_side = choosePreferredReverseSide(goal_pose);
    if (goal_preferred_side.has_value()) {
      preferred_side = *goal_preferred_side;
      RCLCPP_INFO(
        this->logger_,
        "Ackermann recovery goal-relative preference is %s.",
        sideName(preferred_side).c_str());
    } else {
      RCLCPP_WARN(
        this->logger_,
        "Ackermann recovery could not transform the goal pose into %s; defaulting to left reverse.",
        this->local_frame_.c_str());
    }
  }

  const TurnSide alternate_side =
    preferred_side == TurnSide::LEFT ? TurnSide::RIGHT : TurnSide::LEFT;

  if (isReverseSideFeasible(current_pose, preferred_side)) {
    RCLCPP_INFO(
      this->logger_,
      "Ackermann recovery selected reverse side: %s.",
      sideName(preferred_side).c_str());
    return preferred_side;
  }

  if (isReverseSideFeasible(current_pose, alternate_side)) {
    RCLCPP_INFO(
      this->logger_,
      "Ackermann recovery selected reverse side: %s.",
      sideName(alternate_side).c_str());
    return alternate_side;
  }

  return std::nullopt;
}

geometry_msgs::msg::Twist AckermannBackUp::buildReverseCommand(TurnSide side) const
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = reverse_linear_vel_;
  cmd_vel.angular.z = side == TurnSide::LEFT ? -reverse_angular_vel_ : reverse_angular_vel_;
  return cmd_vel;
}

std::string AckermannBackUp::sideName(TurnSide side) const
{
  switch (side) {
    case TurnSide::LEFT:
      return "left";
    case TurnSide::RIGHT:
      return "right";
    default:
      return "UNKNOWN";
  }
}

}  // namespace autonomous_navigation

PLUGINLIB_EXPORT_CLASS(autonomous_navigation::AckermannBackUp, nav2_core::Behavior)
