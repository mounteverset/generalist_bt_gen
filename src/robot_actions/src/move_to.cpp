#include "robot_actions/move_to.hpp"

#include "robot_actions/common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace robot_actions
{

MoveTo::MoveTo(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, config, params)
{
}

BT::PortsList MoveTo::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>("pose", "Target pose as 'x,y,theta'")});
}

namespace
{

std::string trim_copy(const std::string & input)
{
  auto front = std::find_if_not(input.begin(), input.end(), [](unsigned char c) {return std::isspace(c);});
  auto back = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) {return std::isspace(c);}).base();
  if (front >= back) {
    return {};
  }
  return std::string(front, back);
}

bool parse_pose_string(const std::string & raw, double & x, double & y, double & yaw)
{
  std::string cleaned = raw;
  cleaned.erase(std::remove_if(cleaned.begin(), cleaned.end(),
    [](unsigned char c) {return c == '[' || c == ']' || c == '(' || c == ')';}), cleaned.end());

  std::stringstream ss(cleaned);
  std::string token;
  std::vector<double> values;
  while (std::getline(ss, token, ',')) {
    auto trimmed = trim_copy(token);
    if (trimmed.empty()) {
      continue;
    }
    try {
      values.push_back(std::stod(trimmed));
    } catch (const std::exception &) {
      return false;
    }
  }

  if (values.size() < 2) {
    return false;
  }

  x = values[0];
  y = values[1];
  yaw = values.size() >= 3 ? values[2] : 0.0;
  return true;
}

}  // namespace

bool MoveTo::setGoal(Goal & goal)
{
  const auto pose_str = getInput<std::string>("pose").value_or("0,0,0");
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

  if (!parse_pose_string(pose_str, x, y, theta)) {
    RCLCPP_WARN(get_logger(), "MoveTo → Invalid pose string '%s', using defaults.", pose_str.c_str());
  }

  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = std::sin(theta / 2.0);
  goal.pose.pose.orientation.w = std::cos(theta / 2.0);
  goal.pose.header.frame_id = "map";
  if (auto node = node_.lock()) {
    goal.pose.header.stamp = node->get_clock()->now();
  }
  RCLCPP_INFO(get_logger(), "MoveTo → Sending goal (%.2f, %.2f, %.2f)", x, y, theta);
  return true;
}

BT::NodeStatus MoveTo::onResultReceived(const WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "MoveTo → Goal reached.");
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(get_logger(), "MoveTo → Action failed with code %d.", static_cast<int>(result.code));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveTo::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(get_logger(), "MoveTo → action failure: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace robot_actions
