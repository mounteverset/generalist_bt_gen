#include "bt_actions/explore_field_action.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp/exceptions.h"
#include "rclcpp/rclcpp.hpp"

namespace bt_actions
{

namespace
{
constexpr char kBlackboardNodeKey[] = "ros_node";
constexpr char kDefaultWaypointsKey[] = "exploration_waypoints";
constexpr char kDefaultFrameId[] = "map";
constexpr char kDefaultActionName[] = "follow_waypoints";
}  // namespace

ExploreFieldAction::ExploreFieldAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  start_time_(0)
{
}

BT::PortsList ExploreFieldAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("waypoints_key", kDefaultWaypointsKey, "Blackboard key for waypoint list"),
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", "Explicit waypoint list"),
    BT::InputPort<std::string>("action_name", kDefaultActionName, "Name of the FollowWaypoints action server"),
    BT::InputPort<unsigned int>("server_timeout_ms", 1000U, "Milliseconds to wait for server discovery"),
    BT::InputPort<unsigned int>("timeout_ms", 120000U, "Milliseconds before exploration goal times out")
  };
}

BT::NodeStatus ExploreFieldAction::onStart()
{
  auto node = get_node();

  const auto action_name =
    getInput<std::string>("action_name").value_or(std::string{kDefaultActionName});
  if (!client_ || action_name != action_name_) {
    client_ = rclcpp_action::create_client<FollowWaypoints>(node, action_name);
    action_name_ = action_name;
  }

  const auto server_timeout_ms = getInput<unsigned int>("server_timeout_ms").value_or(1000U);
  if (!client_->wait_for_action_server(std::chrono::milliseconds(server_timeout_ms))) {
    RCLCPP_WARN(
      node->get_logger(), "ExploreFieldAction [%s]: action server '%s' unavailable after %u ms",
      name().c_str(), action_name_.c_str(), server_timeout_ms);
    return BT::NodeStatus::FAILURE;
  }

  try {
    current_goal_.poses = resolve_waypoints();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(), "ExploreFieldAction [%s]: %s", name().c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  if (current_goal_.poses.empty()) {
    RCLCPP_WARN(
      node->get_logger(), "ExploreFieldAction [%s]: waypoint list is empty",
      name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = node->now();
  goal_handle_.reset();
  goal_future_ = {};
  result_future_ = {};

  rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
  goal_future_ = client_->async_send_goal(current_goal_, options);
  if (!goal_future_.valid()) {
    RCLCPP_ERROR(
      node->get_logger(), "ExploreFieldAction [%s]: failed to send goal", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node->get_logger(), "ExploreFieldAction [%s]: sent waypoint goal with %zu poses",
    name().c_str(), current_goal_.poses.size());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExploreFieldAction::onRunning()
{
  auto node = get_node();
  rclcpp::spin_some(node);

  if (!goal_future_.valid()) {
    RCLCPP_ERROR(
      node->get_logger(), "ExploreFieldAction [%s]: goal future invalid", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_handle_) {
    const auto status = goal_future_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
      goal_handle_ = goal_future_.get();
      if (!goal_handle_) {
        RCLCPP_WARN(
          node->get_logger(), "ExploreFieldAction [%s]: goal rejected by server",
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
            node->get_logger(), "ExploreFieldAction [%s]: goal succeeded", name().c_str());
          config().blackboard->set("last_exploration_goal", current_goal_);
          goal_handle_.reset();
          result_future_ = {};
          return BT::NodeStatus::SUCCESS;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(
            node->get_logger(), "ExploreFieldAction [%s]: goal canceled", name().c_str());
          goal_handle_.reset();
          result_future_ = {};
          return BT::NodeStatus::FAILURE;
        default:
          RCLCPP_WARN(
            node->get_logger(),
            "ExploreFieldAction [%s]: goal failed with result code %d",
            name().c_str(), static_cast<int>(wrapped_result.code));
          goal_handle_.reset();
          result_future_ = {};
          return BT::NodeStatus::FAILURE;
      }
    }
  }

  const auto timeout_ms = getInput<unsigned int>("timeout_ms").value_or(120000U);
  if (timeout_ms > 0U) {
    const rclcpp::Duration timeout{std::chrono::milliseconds(timeout_ms)};
    if (node->now() - start_time_ > timeout) {
      RCLCPP_WARN(
        node->get_logger(),
        "ExploreFieldAction [%s]: goal timed out after %u ms",
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

void ExploreFieldAction::onHalted()
{
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  goal_future_ = {};
  result_future_ = {};
}

rclcpp::Node::SharedPtr ExploreFieldAction::get_node()
{
  if (!node_) {
    try {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>(kBlackboardNodeKey);
    } catch (const BT::RuntimeError &) {
      throw std::runtime_error(
              "ExploreFieldAction requires a rclcpp::Node shared pointer stored in blackboard key '" +
              std::string{kBlackboardNodeKey} + "'");
    }
  }
  return node_;
}

std::vector<geometry_msgs::msg::PoseStamped> ExploreFieldAction::resolve_waypoints()
{
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;

  if (auto input = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints")) {
    waypoints = input.value();
  } else {
    const auto key =
      getInput<std::string>("waypoints_key").value_or(std::string{kDefaultWaypointsKey});
    if (!config().blackboard->get(key, waypoints)) {
      std::vector<geometry_msgs::msg::Pose> poses;
      if (config().blackboard->get(key, poses)) {
        waypoints.reserve(poses.size());
        auto node = get_node();
        for (const auto & pose : poses) {
          geometry_msgs::msg::PoseStamped stamped;
          stamped.pose = pose;
          stamped.header.frame_id = kDefaultFrameId;
          stamped.header.stamp = node->now();
          waypoints.push_back(stamped);
        }
      } else {
        throw std::runtime_error(
                "ExploreFieldAction failed to find waypoints for key '" + key + "' on the blackboard");
      }
    }
  }

  if (waypoints.empty()) {
    throw std::runtime_error("ExploreFieldAction received an empty waypoint list");
  }

  auto node = get_node();
  for (auto & pose : waypoints) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = kDefaultFrameId;
    }
    if (pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) {
      pose.header.stamp = node->now();
    }
  }

  return waypoints;
}

}  // namespace bt_actions
