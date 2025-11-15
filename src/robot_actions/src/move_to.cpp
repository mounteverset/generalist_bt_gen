#include "robot_actions/move_to.hpp"

#include "robot_actions/common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sstream>

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

bool MoveTo::setGoal(Goal & goal)
{
  const auto pose_str = getInput<std::string>("pose").value_or("0,0,0");
  std::stringstream ss(pose_str);
  double x = 0.0, y = 0.0, theta = 0.0;
  char comma;
  if (!(ss >> x >> comma >> y >> comma >> theta)) {
    RCLCPP_WARN(get_logger(), "MoveTo → Invalid pose string '%s', using defaults.", pose_str.c_str());
  }

  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = sin(theta / 2.0);
  goal.pose.pose.orientation.w = cos(theta / 2.0);
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
