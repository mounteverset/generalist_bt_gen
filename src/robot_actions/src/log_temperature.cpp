#include "robot_actions/log_temperature.hpp"

#include "robot_actions/common.hpp"

#include <fstream>

namespace robot_actions
{

LogTemperature::LogTemperature(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosServiceNode<std_srvs::srv::Trigger>(name, config, params)
{
  enable_debug_logging_ = is_debug_logging_enabled(node_.lock());
}

BT::PortsList LogTemperature::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>("logfile_path", "/tmp/temperature_log.txt")});
}

bool LogTemperature::setRequest(Request::SharedPtr & request)
{
  current_log_path_ = getInput<std::string>("logfile_path").value_or("/tmp/temperature_log.txt");
  (void)request;
  if (enable_debug_logging_) {
    RCLCPP_INFO(get_logger(), "LogTemperature → requesting measurement for %s", current_log_path_.c_str());
  }
  return true;
}

BT::NodeStatus LogTemperature::onResponseReceived(const Response::SharedPtr & response)
{
  if (enable_debug_logging_) {
    RCLCPP_INFO(
      get_logger(), "LogTemperature → response: success=%d message=%s",
      response->success, response->message.c_str());
  }
  if (!response->success) {
    return BT::NodeStatus::FAILURE;
  }

  std::ofstream logfile(current_log_path_, std::ios::app);
  logfile << response->message << '\n';
  if (!logfile) {
    RCLCPP_ERROR(
      get_logger(), "LogTemperature → failed to append measurement to %s",
      current_log_path_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LogTemperature::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(get_logger(), "LogTemperature → service failure: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace robot_actions
