#include "robot_actions/take_picture.hpp"

#include "robot_actions/common.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace robot_actions
{

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
  auto now = std::chrono::system_clock::now();
  auto now_time = std::chrono::system_clock::to_time_t(now);
  std::stringstream path;
  path << "/tmp/photo_" << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S") << ".jpg";
  setOutput("filepath", path.str());

  RCLCPP_INFO(get_logger(), "TakePicture → stored image at %s", path.str().c_str());
  // TODO: call camera service
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_actions
