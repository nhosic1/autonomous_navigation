#ifndef AUTONOMOUS_NAVIGATION__NAV2_ACKERMANN_BACK_UP_BEHAVIOR_HPP_
#define AUTONOMOUS_NAVIGATION__NAV2_ACKERMANN_BACK_UP_BEHAVIOR_HPP_

#include <memory>
#include <optional>
#include <string>

#include "autonomous_navigation/action/ackermann_back_up.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_behaviors/timed_behavior.hpp>

namespace autonomous_navigation
{

class AckermannBackUp
: public nav2_behaviors::TimedBehavior<autonomous_navigation::action::AckermannBackUp>
{
public:
  using ActionT = autonomous_navigation::action::AckermannBackUp;
  using CostmapInfoType = nav2_core::CostmapInfoType;
  using ResultStatus = nav2_behaviors::ResultStatus;
  using Status = nav2_behaviors::Status;

  AckermannBackUp();
  ~AckermannBackUp() override = default;

  ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> command) override;
  ResultStatus onCycleUpdate() override;
  CostmapInfoType getResourceInfo() override;

  void onConfigure() override;

private:
  enum class TurnSide
  {
    LEFT,
    RIGHT
  };

  bool getCurrentPose(geometry_msgs::msg::PoseStamped & pose) const;
  bool isGoalPoseValid(const geometry_msgs::msg::PoseStamped & goal_pose) const;
  std::optional<TurnSide> choosePreferredReverseSide(
    const geometry_msgs::msg::PoseStamped & goal_pose) const;
  double yawDeltaFromPhaseStart(const geometry_msgs::msg::PoseStamped & pose) const;
  bool isCommandCollisionFree(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & cmd_vel,
    double simulation_time) const;
  bool isForwardEscapeFeasible(const geometry_msgs::msg::PoseStamped & current_pose) const;
  bool isReverseSideFeasible(
    const geometry_msgs::msg::PoseStamped & current_pose, TurnSide side) const;
  std::optional<TurnSide> selectReverseSide(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose) const;
  geometry_msgs::msg::Twist buildReverseCommand(TurnSide side) const;
  std::string sideName(TurnSide side) const;

  typename ActionT::Feedback::SharedPtr feedback_;
  geometry_msgs::msg::PoseStamped phase_start_pose_;

  TurnSide selected_side_{TurnSide::LEFT};
  double reverse_linear_vel_{0.0};
  double reverse_angular_vel_{0.0};
  double forward_linear_vel_{0.0};
  double forward_angular_vel_{0.0};
  double simulate_ahead_time_{2.0};
  double forward_probe_time_{1.0};
  double sample_time_step_{0.1};
  double max_reverse_heading_change_rad_{0.0};
  double min_reverse_heading_change_rad_{0.0};
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_{0, 0, RCL_STEADY_TIME};
};

}  // namespace autonomous_navigation

#endif  // AUTONOMOUS_NAVIGATION__NAV2_ACKERMANN_BACK_UP_BEHAVIOR_HPP_
