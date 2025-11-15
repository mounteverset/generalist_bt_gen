#include "robot_actions/log_temperature.hpp"

#include "robot_actions/common.hpp"

namespace robot_actions
{

LogTemperature::LogTemperature(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosServiceNode<std_srvs::srv::Trigger>(name, config, params)
{
}

BT::PortsList LogTemperature::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>("logfile_path", "/tmp/temperature_log.txt")});
}

bool LogTemperature::setRequest(Request::SharedPtr & request)
{
  current_log_path_ = getInput<std::string>("logfile_path").value_or("/tmp/temperature_log.txt");
  (void)request;
  RCLCPP_INFO(get_logger(), "LogTemperature → requesting measurement for %s", current_log_path_.c_str());
  return true;
}

BT::NodeStatus LogTemperature::onResponseReceived(const Response::SharedPtr & response)
{
  RCLCPP_INFO(
    get_logger(), "LogTemperature → response: success=%d message=%s",
    response->success, response->message.c_str());
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus LogTemperature::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(get_logger(), "LogTemperature → service failure: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace robot_actions
