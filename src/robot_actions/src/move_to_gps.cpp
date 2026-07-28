#include "robot_actions/move_to_gps.hpp"

#include "robot_actions/common.hpp"
#include "robot_actions/gps_waypoint_utils.hpp"

#include <cmath>
#include <string>

namespace robot_actions
{

MoveToGPS::MoveToGPS(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosActionNode<nav2_msgs::action::FollowGPSWaypoints>(name, config, params)
{
  if (auto node = node_.lock()) {
    enable_debug_logging_ = is_debug_logging_enabled(node);
  }
}

BT::PortsList MoveToGPS::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::string>(
        "gps_pose", "GPS target as lat,lon[,yaw] or lat,lon,alt,yaw"),
    });
}

bool MoveToGPS::setGoal(Goal & goal)
{
  const auto raw_pose = getInput<std::string>("gps_pose").value_or("");
  GpsWaypoint waypoint;
  if (!parse_gps_waypoint(raw_pose, waypoint)) {
    RCLCPP_ERROR(
      get_logger(),
      "MoveToGPS -> invalid GPS pose '%s'; expected lat,lon[,yaw] or lat,lon,alt,yaw",
      raw_pose.c_str());
    return false;
  }

  geographic_msgs::msg::GeoPose pose;
  pose.position.latitude = waypoint.latitude;
  pose.position.longitude = waypoint.longitude;
  pose.position.altitude = waypoint.altitude;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(waypoint.yaw / 2.0);
  pose.orientation.w = std::cos(waypoint.yaw / 2.0);

  goal.number_of_loops = 0;
  goal.goal_index = 0;
  goal.gps_poses = {pose};
  last_latitude_ = waypoint.latitude;
  last_longitude_ = waypoint.longitude;

  if (enable_debug_logging_) {
    RCLCPP_INFO(
      get_logger(),
      "MoveToGPS -> sending GPS goal (lat=%.8f, lon=%.8f, alt=%.2f, yaw=%.3f) action='%s'",
      waypoint.latitude,
      waypoint.longitude,
      waypoint.altitude,
      waypoint.yaw,
      action_name_.c_str());
  }
  return true;
}

BT::NodeStatus MoveToGPS::onFeedback(std::shared_ptr<const Feedback> feedback)
{
  if (feedback && enable_debug_logging_) {
    RCLCPP_DEBUG(
      get_logger(),
      "MoveToGPS -> current waypoint index %u",
      feedback->current_waypoint);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToGPS::onResultReceived(const WrappedResult & result)
{
  if (
    result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    result.result &&
    result.result->error_code == nav2_msgs::action::FollowGPSWaypoints::Result::NONE &&
    result.result->missed_waypoints.empty())
  {
    RCLCPP_INFO(
      get_logger(),
      "MoveToGPS -> GPS goal reached (lat=%.8f, lon=%.8f).",
      last_latitude_,
      last_longitude_);
    return BT::NodeStatus::SUCCESS;
  }

  if (result.result) {
    RCLCPP_ERROR(
      get_logger(),
      "MoveToGPS -> GPS action failed (result=%d, error=%d, message='%s', missed=%zu).",
      static_cast<int>(result.code),
      static_cast<int>(result.result->error_code),
      result.result->error_msg.c_str(),
      result.result->missed_waypoints.size());
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToGPS::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(
    get_logger(),
    "MoveToGPS -> action failure: %s (lat=%.8f, lon=%.8f, action='%s').",
    BT::toStr(error),
    last_latitude_,
    last_longitude_,
    action_name_.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToGPS::onFailure(
  BT::ActionNodeErrorCode error,
  const std::optional<WrappedResult> & result)
{
  if (result && result->result) {
    RCLCPP_ERROR(
      get_logger(),
      "MoveToGPS -> action error context: code=%d message='%s'.",
      static_cast<int>(result->result->error_code),
      result->result->error_msg.c_str());
  }
  return onFailure(error);
}

void MoveToGPS::onHalt()
{
  RCLCPP_WARN(get_logger(), "MoveToGPS -> halt requested, canceling GPS goal.");
}

}  // namespace robot_actions
