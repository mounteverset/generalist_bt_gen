#include "robot_actions/find_object_location.hpp"

#include "robot_actions/common.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace robot_actions
{

FindObjectLocation::FindObjectLocation(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosServiceNode<language_feature_msgs::srv::FindObjectLocations>(name, config, params)
{
  enable_debug_logging_ = is_debug_logging_enabled(node_.lock());
}

BT::PortsList FindObjectLocation::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::string>(
        "object", "Object query string passed to FindAnything."),
      BT::InputPort<int>(
        "max_results", 1, "Maximum FindAnything locations to request."),
      BT::InputPort<double>(
        "default_yaw", 0.0,
        "Yaw used for the 2D pose because FindAnything returns point locations."),
      BT::OutputPort<double>("x", "Object location x coordinate."),
      BT::OutputPort<double>("y", "Object location y coordinate."),
      BT::OutputPort<double>("yaw", "Output yaw in radians."),
      BT::OutputPort<std::string>("pose", "Object location formatted as 'x,y,yaw'."),
      BT::OutputPort<std::string>("frame_id", "Frame id of the returned point.")
    });
}

bool FindObjectLocation::setRequest(Request::SharedPtr & request)
{
  query_ = getInput<std::string>("object").value_or("");
  yaw_ = getInput<double>("default_yaw").value_or(0.0);
  const int max_results = std::clamp(getInput<int>("max_results").value_or(1), 1, 50);

  if (query_.empty()) {
    RCLCPP_ERROR(get_logger(), "FindObjectLocation -> missing required input port 'object'.");
    return false;
  }

  request->query = query_;
  request->max_results = static_cast<uint32_t>(max_results);

  if (enable_debug_logging_) {
    RCLCPP_INFO(
      get_logger(), "FindObjectLocation -> querying '%s' with max_results=%d",
      query_.c_str(), max_results);
  }
  return true;
}

BT::NodeStatus FindObjectLocation::onResponseReceived(const Response::SharedPtr & response)
{
  if (!response || response->locations.empty()) {
    RCLCPP_WARN(
      get_logger(), "FindObjectLocation -> no location returned for query '%s'.",
      query_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto & location = response->locations.front();
  const double x = location.point.x;
  const double y = location.point.y;

  std::ostringstream pose_stream;
  pose_stream << std::fixed << std::setprecision(3) << x << "," << y << "," << yaw_;
  const auto pose = pose_stream.str();

  setOutput("x", x);
  setOutput("y", y);
  setOutput("yaw", yaw_);
  setOutput("pose", pose);
  setOutput("frame_id", location.header.frame_id);

  if (enable_debug_logging_) {
    RCLCPP_INFO(
      get_logger(),
      "FindObjectLocation -> '%s' resolved to pose=%s frame='%s'",
      query_.c_str(), pose.c_str(), location.header.frame_id.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FindObjectLocation::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(get_logger(), "FindObjectLocation -> service failure: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace robot_actions
