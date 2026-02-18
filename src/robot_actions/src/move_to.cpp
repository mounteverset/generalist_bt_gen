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
  auto node = node_.lock();
  if (!node) {
    return;
  }

  if (node->has_parameter("nav2_goal_frame_id")) {
    default_goal_frame_id_ = node->get_parameter("nav2_goal_frame_id").as_string();
  } else {
    default_goal_frame_id_ = node->declare_parameter<std::string>(
      "nav2_goal_frame_id", "map");
  }

  if (default_goal_frame_id_.empty()) {
    default_goal_frame_id_ = "map";
  }
  while (!default_goal_frame_id_.empty() && default_goal_frame_id_.front() == '/') {
    default_goal_frame_id_.erase(default_goal_frame_id_.begin());
  }

  RCLCPP_INFO(
    get_logger(),
    "MoveTo -> using default goal frame '%s' (action='%s')",
    default_goal_frame_id_.c_str(), action_name_.c_str());
}

BT::PortsList MoveTo::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::string>("pose", "Target pose as 'x,y,theta'"),
      BT::InputPort<std::string>(
        "frame_id", "",
        "Target frame id for NavigateToPose. Defaults to nav2_goal_frame_id or derived namespace."),
    });
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
  auto frame_id = trim_copy(getInput<std::string>("frame_id").value_or(""));
  if (frame_id.empty()) {
    frame_id = default_goal_frame_id_;
  }
  while (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  if (frame_id.empty()) {
    frame_id = "map";
  }

  RCLCPP_DEBUG(get_logger(), "MoveTo -> setGoal pose='%s'", pose_str.c_str());
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

  if (!parse_pose_string(pose_str, x, y, theta)) {
    RCLCPP_WARN(get_logger(), "MoveTo → Invalid pose string '%s', using defaults.", pose_str.c_str());
  }

  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.x = 0.0;
  goal.pose.pose.orientation.y = 0.0;
  goal.pose.pose.orientation.z = std::sin(theta / 2.0);
  goal.pose.pose.orientation.w = std::cos(theta / 2.0);
  goal.pose.header.frame_id = frame_id;
  if (auto node = node_.lock()) {
    goal.pose.header.stamp = node->get_clock()->now();
  }
  last_goal_frame_id_ = frame_id;
  last_goal_x_ = x;
  last_goal_y_ = y;
  last_goal_theta_ = theta;
  RCLCPP_INFO(
    get_logger(), "MoveTo → Sending goal (%.2f, %.2f, %.2f) frame='%s'",
    x, y, theta, frame_id.c_str());
  return true;
}

BT::NodeStatus MoveTo::onFeedback(std::shared_ptr<const Feedback> feedback)
{
  if (!feedback) {
    RCLCPP_DEBUG(get_logger(), "MoveTo -> feedback: <null>");
    return BT::NodeStatus::RUNNING;
  }

  const double nav_time =
    static_cast<double>(feedback->navigation_time.sec) +
    static_cast<double>(feedback->navigation_time.nanosec) * 1e-9;
  const double eta =
    static_cast<double>(feedback->estimated_time_remaining.sec) +
    static_cast<double>(feedback->estimated_time_remaining.nanosec) * 1e-9;
  RCLCPP_DEBUG(
    get_logger(),
    "MoveTo -> feedback: dist=%.2f recoveries=%d nav_time=%.2f eta=%.2f",
    feedback->distance_remaining,
    static_cast<int>(feedback->number_of_recoveries),
    nav_time,
    eta);
  return BT::NodeStatus::RUNNING;
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
  if (error == BT::ActionNodeErrorCode::GOAL_REJECTED_BY_SERVER) {
    RCLCPP_ERROR(
      get_logger(),
      "MoveTo → action failure: %s (pose=%.3f,%.3f,%.3f frame='%s' action='%s'). "
      "This usually means the navigation server rejected the goal frame or current state.",
      BT::toStr(error), last_goal_x_, last_goal_y_, last_goal_theta_,
      last_goal_frame_id_.c_str(), action_name_.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "MoveTo → action failure: %s", BT::toStr(error));
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveTo::onFailure(
  BT::ActionNodeErrorCode error,
  const std::optional<WrappedResult> & result)
{
  if (result && result->result) {
    RCLCPP_ERROR(
      get_logger(),
      "MoveTo → failure context: action_code=%d nav2_error_code=%u nav2_error_msg='%s'",
      static_cast<int>(result->code),
      static_cast<unsigned int>(result->result->error_code),
      result->result->error_msg.c_str());
  }
  return onFailure(error);
}

void MoveTo::onHalt()
{
  RCLCPP_WARN(get_logger(), "MoveTo -> halt requested, canceling goal.");
}

}  // namespace robot_actions
