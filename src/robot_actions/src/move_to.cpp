#include "robot_actions/move_to.hpp"

#include "robot_actions/common.hpp"

namespace robot_actions
{

MoveTo::MoveTo(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList MoveTo::providedPorts()
{
  return {BT::InputPort<std::string>("pose", "Target pose (e.g., x,y,yaw or frame id).")};
}

BT::NodeStatus MoveTo::tick()
{
  const auto pose = getInput<std::string>("pose").value_or("unknown");
  RCLCPP_INFO(get_logger(), "MoveTo → navigating to %s", pose.c_str());
  // TODO: Integrate Nav2 action client
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_actions
