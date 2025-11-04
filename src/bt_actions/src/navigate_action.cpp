#include "bt_actions/navigate_action.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp/exceptions.h"
#include "rclcpp/rclcpp.hpp"

namespace bt_actions
{

namespace
{
constexpr char kDefaultActionName[] = "navigate_to_pose";
constexpr char kBlackboardNodeKey[] = "ros_node";
constexpr char kDefaultGoalFrame[] = "map";
}  // namespace

NavigateAction::NavigateAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  start_time_(0)
{
}

BT::PortsList NavigateAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("waypoint"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<std::string>("action_name", kDefaultActionName, "Name of the Nav2 action server"),
    BT::InputPort<unsigned int>("server_timeout_ms", 1000U, "Milliseconds to wait for server discovery"),
    BT::InputPort<unsigned int>("timeout_ms", 60000U, "Milliseconds before navigation goal times out")
  };
}

BT::NodeStatus NavigateAction::onStart()
{
  auto node = get_node();

  const auto action_name =
    getInput<std::string>("action_name").value_or(std::string{kDefaultActionName});
  if (!client_ || action_name != action_name_) {
    client_ = rclcpp_action::create_client<NavigateToPose>(node, action_name);
    action_name_ = action_name;
  }

  const auto server_timeout_ms = getInput<unsigned int>("server_timeout_ms").value_or(1000U);
  if (!client_->wait_for_action_server(std::chrono::milliseconds(server_timeout_ms))) {
    RCLCPP_WARN(
      node->get_logger(), "NavigateAction [%s]: action server '%s' unavailable after %u ms",
      name().c_str(), action_name_.c_str(), server_timeout_ms);
    return BT::NodeStatus::FAILURE;
  }

  try {
    current_goal_.pose = resolve_goal_pose();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(), "NavigateAction [%s]: %s", name().c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = node->now();
  goal_handle_.reset();
  goal_future_ = {};
  result_future_ = {};

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  goal_future_ = client_->async_send_goal(current_goal_, options);
  if (!goal_future_.valid()) {
    RCLCPP_ERROR(
      node->get_logger(), "NavigateAction [%s]: failed to send goal", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node->get_logger(), "NavigateAction [%s]: sent goal to '%s'",
    name().c_str(), action_name_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateAction::onRunning()
{
  auto node = get_node();
  rclcpp::spin_some(node);

  if (!goal_future_.valid()) {
    RCLCPP_ERROR(
      node->get_logger(), "NavigateAction [%s]: goal future invalid", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_handle_) {
    const auto status = goal_future_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
      goal_handle_ = goal_future_.get();
      if (!goal_handle_) {
        RCLCPP_WARN(
          node->get_logger(), "NavigateAction [%s]: goal rejected by server",
          name().c_str());
        return BT::NodeStatus::FAILURE;
      }
      result_future_ = client_->async_get_result(goal_handle_);
    }
    return BT::NodeStatus::RUNNING;
  }

  if (result_future_.valid()) {
    const auto status = result_future_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
      const auto wrapped_result = result_future_.get();
      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(
            node->get_logger(), "NavigateAction [%s]: goal succeeded", name().c_str());
          goal_handle_.reset();
          result_future_ = {};
          config().blackboard->set("last_navigation_goal", current_goal_);
          return BT::NodeStatus::SUCCESS;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(
            node->get_logger(), "NavigateAction [%s]: goal canceled", name().c_str());
          goal_handle_.reset();
          result_future_ = {};
          return BT::NodeStatus::FAILURE;
        default:
          RCLCPP_WARN(
            node->get_logger(),
            "NavigateAction [%s]: goal failed with result code %d",
            name().c_str(), static_cast<int>(wrapped_result.code));
          goal_handle_.reset();
          result_future_ = {};
          return BT::NodeStatus::FAILURE;
      }
    }
  }

  const auto timeout_ms = getInput<unsigned int>("timeout_ms").value_or(60000U);
  if (timeout_ms > 0U) {
    const rclcpp::Duration timeout{std::chrono::milliseconds(timeout_ms)};
    if (node->now() - start_time_ > timeout) {
      RCLCPP_WARN(
        node->get_logger(), "NavigateAction [%s]: goal timed out after %u ms",
        name().c_str(), timeout_ms);
      if (goal_handle_) {
        client_->async_cancel_goal(goal_handle_);
      }
      goal_handle_.reset();
      result_future_ = {};
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void NavigateAction::onHalted()
{
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  goal_future_ = {};
  result_future_ = {};
}

rclcpp::Node::SharedPtr NavigateAction::get_node()
{
  if (!node_) {
    try {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>(kBlackboardNodeKey);
    } catch (const BT::RuntimeError &) {
      throw std::runtime_error(
              "NavigateAction requires a rclcpp::Node shared pointer stored in blackboard key '" +
              std::string{kBlackboardNodeKey} + "'");
    }
  }
  return node_;
}

geometry_msgs::msg::PoseStamped NavigateAction::resolve_goal_pose()
{
  geometry_msgs::msg::PoseStamped goal;
  if (auto goal_input = getInput<geometry_msgs::msg::PoseStamped>("goal")) {
    goal = goal_input.value();
  } else {
    auto waypoint_input = getInput<std::string>("waypoint");
    if (!waypoint_input) {
      throw std::runtime_error(
              "NavigateAction requires either 'goal' PoseStamped input port or 'waypoint' string");
    }
    const auto waypoint_key = waypoint_input.value();
    if (!config().blackboard->get(waypoint_key, goal)) {
      throw std::runtime_error(
              "NavigateAction could not find waypoint '" + waypoint_key + "' on the blackboard");
    }
  }

  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = kDefaultGoalFrame;
  }
  goal.header.stamp = get_node()->now();
  return goal;
}

}  // namespace bt_actions
