#include "robot_actions/restart_node.hpp"

#include "robot_actions/common.hpp"

namespace robot_actions
{

RestartNode::RestartNode(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList RestartNode::providedPorts()
{
  return {BT::InputPort<std::string>("node_name")};
}

BT::NodeStatus RestartNode::tick()
{
  const auto node_name = getInput<std::string>("node_name").value_or("unknown_node");
  RCLCPP_WARN(get_logger(), "RestartNode → would restart ROS node: %s", node_name.c_str());
  // TODO: call lifecycle manager / supervisor service.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_actions
