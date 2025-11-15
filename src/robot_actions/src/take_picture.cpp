#include "robot_actions/take_picture.hpp"

#include "robot_actions/common.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace robot_actions
{

TakePicture::TakePicture(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosServiceNode<std_srvs::srv::Trigger>(name, config, params)
{
}

BT::PortsList TakePicture::providedPorts()
{
  return providedBasicPorts({BT::OutputPort<std::string>("filepath")});
}

bool TakePicture::setRequest(Request::SharedPtr & request)
{
  (void)request;
  RCLCPP_INFO(get_logger(), "TakePicture → requesting camera capture.");
  return true;
}

BT::NodeStatus TakePicture::onResponseReceived(const Response::SharedPtr & response)
{
  std::string filepath = response->message;
  if (filepath.empty()) {
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream path;
    path << "/tmp/photo_" << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S") << ".jpg";
    filepath = path.str();
  }
  setOutput("filepath", filepath);
  RCLCPP_INFO(get_logger(), "TakePicture → received path %s", filepath.c_str());
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus TakePicture::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(get_logger(), "TakePicture → service failure: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace robot_actions
