#include "autonomous_navigation/nav2/nav2_ackermann_back_up_action_node.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/duration.hpp>

namespace autonomous_navigation
{

AckermannBackUpActionNode::AckermannBackUpActionNode(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void AckermannBackUpActionNode::on_tick()
{
  double time_allowance_seconds = 10.0;
  getInput("time_allowance", time_allowance_seconds);
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance_seconds);

  geometry_msgs::msg::PoseStamped goal_pose;
  if (getInput("goal", goal_pose)) {
    goal_.goal_pose = goal_pose;
  } else {
    goal_.goal_pose = geometry_msgs::msg::PoseStamped();
  }
}

}  // namespace autonomous_navigation

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<autonomous_navigation::AckermannBackUpActionNode>(
        name, "ackermann_backup", config);
    };

  factory.registerBuilder(
    BT::CreateManifest<autonomous_navigation::AckermannBackUpActionNode>("AckermannBackUp"),
    builder);
}
