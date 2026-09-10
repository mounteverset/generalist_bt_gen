#include "robot_actions/set_depth.hpp"

#include "robot_actions/common.hpp"

namespace robot_actions
{

SetDepth::SetDepth(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosActionNode<stepper_interfaces::action::SetDepth>(name, config, params)
{
  enable_debug_logging_ = is_debug_logging_enabled(node_.lock());
}

BT::PortsList SetDepth::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<int>(
        "target_depth_cm", "Absolute probe depth in centimeters (0 = raised)."),
      BT::InputPort<int>("max_depth_cm", 500, "Maximum permitted probe depth."),
    });
}

bool SetDepth::setGoal(Goal & goal)
{
  const auto target = getInput<int>("target_depth_cm");
  const auto maximum = getInput<int>("max_depth_cm");
  if (!target || !maximum || *maximum < 0 || *target < 0 || *target > *maximum) {
    RCLCPP_ERROR(
      get_logger(), "SetDepth -> target_depth_cm must be within the configured limit");
    return false;
  }

  goal.target_depth_cm = *target;
  last_target_depth_cm_ = goal.target_depth_cm;
  if (enable_debug_logging_) {
    RCLCPP_INFO(
      get_logger(), "SetDepth -> moving probe to %d cm (action='%s')",
      last_target_depth_cm_, action_name_.c_str());
  }
  return true;
}

BT::NodeStatus SetDepth::onFeedback(std::shared_ptr<const Feedback> feedback)
{
  if (feedback && enable_debug_logging_) {
    RCLCPP_DEBUG(
      get_logger(), "SetDepth -> current=%d cm progress=%.1f%%",
      feedback->current_depth_cm, feedback->progress_percent);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetDepth::onResultReceived(const WrappedResult & result)
{
  if (
    result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
    result.result->success)
  {
    RCLCPP_INFO(
      get_logger(), "SetDepth -> reached %d cm: %s",
      result.result->final_depth_cm, result.result->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (result.result) {
    RCLCPP_ERROR(
      get_logger(), "SetDepth -> failed at %d cm: %s",
      result.result->final_depth_cm, result.result->message.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "SetDepth -> action returned no result.");
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SetDepth::onFailure(
  BT::ActionNodeErrorCode error,
  const std::optional<WrappedResult> & result)
{
  if (result && result->result) {
    RCLCPP_ERROR(
      get_logger(), "SetDepth -> action failure for %d cm: %s",
      last_target_depth_cm_, result->result->message.c_str());
  }
  return onFailure(error);
}

BT::NodeStatus SetDepth::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(
    get_logger(), "SetDepth -> action failure for %d cm: %s",
    last_target_depth_cm_, BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

void SetDepth::onHalt()
{
  RCLCPP_WARN(get_logger(), "SetDepth -> halt requested, canceling depth goal.");
}

}  // namespace robot_actions
