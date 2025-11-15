#include "robot_actions/basic_actions.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace robot_actions
{

MoveTo::MoveTo(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList MoveTo::providedPorts()
{
  return {BT::InputPort<std::string>("pose", "Target pose JSON or ID")};
}

BT::NodeStatus MoveTo::tick()
{
  auto pose = getInput<std::string>("pose").value_or("unknown");
  RCLCPP_INFO(get_logger(), "MoveTo → navigating to pose: %s", pose.c_str());
  // TODO: Call Nav2 action when available.
  return BT::NodeStatus::SUCCESS;
}

TakePicture::TakePicture(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList TakePicture::providedPorts()
{
  return {BT::OutputPort<std::string>("filepath")};
}

BT::NodeStatus TakePicture::tick()
{
  std::stringstream ss;
  auto now = std::chrono::system_clock::now();
  auto now_time = std::chrono::system_clock::to_time_t(now);
  ss << "/tmp/photo_" << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S") << ".jpg";
  auto path = ss.str();
  setOutput("filepath", path);
  RCLCPP_INFO(get_logger(), "TakePicture → captured photo at %s", path.c_str());
  return BT::NodeStatus::SUCCESS;
}

LogTemperature::LogTemperature(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList LogTemperature::providedPorts()
{
  return {BT::InputPort<std::string>("logfile_path", "/tmp/temperature_log.txt")};
}

BT::NodeStatus LogTemperature::tick()
{
  auto path = getInput<std::string>("logfile_path").value_or("/tmp/temperature_log.txt");
  RCLCPP_INFO(get_logger(), "LogTemperature → appending reading to %s", path.c_str());
  return BT::NodeStatus::SUCCESS;
}

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
  auto node_name = getInput<std::string>("node_name").value_or("unknown_node");
  RCLCPP_WARN(get_logger(), "RestartNode → would restart node: %s", node_name.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_actions

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_actions::MoveTo>("MoveTo");
  factory.registerNodeType<robot_actions::TakePicture>("TakePicture");
  factory.registerNodeType<robot_actions::LogTemperature>("LogTemperature");
  factory.registerNodeType<robot_actions::RestartNode>("RestartNode");
}
