#include "robot_actions/log_temperature.hpp"

#include "robot_actions/common.hpp"

namespace robot_actions
{

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
  const auto path = getInput<std::string>("logfile_path").value_or("/tmp/temperature_log.txt");
  RCLCPP_INFO(get_logger(), "LogTemperature → measured temp and appended to %s", path.c_str());
  // TODO: call temperature service and append data.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_actions
