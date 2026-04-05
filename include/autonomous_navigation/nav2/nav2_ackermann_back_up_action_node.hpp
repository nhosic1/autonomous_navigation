#ifndef AUTONOMOUS_NAVIGATION__NAV2_ACKERMANN_BACK_UP_ACTION_NODE_HPP_
#define AUTONOMOUS_NAVIGATION__NAV2_ACKERMANN_BACK_UP_ACTION_NODE_HPP_

#include <string>

#include "autonomous_navigation/action/ackermann_back_up.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_behavior_tree/bt_action_node.hpp>

namespace autonomous_navigation
{

class AckermannBackUpActionNode
: public nav2_behavior_tree::BtActionNode<autonomous_navigation::action::AckermannBackUp>
{
  using Action = autonomous_navigation::action::AckermannBackUp;

public:
  AckermannBackUpActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for Ackermann backing up"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal pose")
      });
  }

  void on_tick() override;
};

}  // namespace autonomous_navigation

#endif  // AUTONOMOUS_NAVIGATION__NAV2_ACKERMANN_BACK_UP_ACTION_NODE_HPP_
