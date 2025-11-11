#include "bt_executor/bt_executor.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/blackboard.h"
#include "bt_executor/bt_failure_handler.hpp"
#include "bt_executor/bt_monitor.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_executor
{

namespace
{
constexpr char kPublishFeedbackParam[] = "publish_feedback";
}  // namespace

LLMTreeServer::LLMTreeServer(const rclcpp::NodeOptions & options)
: BT::TreeExecutionServer(options),
  logger_{node()->get_logger()}
{
  node()->declare_parameter<bool>(kPublishFeedbackParam, true);
  publish_feedback_ = node()->get_parameter(kPublishFeedbackParam).as_bool();

  monitor_ = std::make_unique<BTMonitor>(*node());
  failure_handler_ = std::make_unique<BTFailureHandler>(*node());

  executeRegistration();
}

void LLMTreeServer::onTreeCreated(BT::Tree & tree)
{
  active_tree_ = &tree;
  last_tick_time_.reset();
  failure_handler_->reset();
  update_blackboard(tree);
  monitor_->publish_status(tree);
}

void LLMTreeServer::update_blackboard(BT::Tree & tree)
{
  if (auto blackboard = tree.rootBlackboard()) {
    blackboard->set("node", node());
    blackboard->set("global_blackboard", globalBlackboard());
  }
}

std::optional<BT::NodeStatus> LLMTreeServer::onLoopAfterTick(BT::NodeStatus status)
{
  const auto now = std::chrono::steady_clock::now();
  if (last_tick_time_) {
    const auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - *last_tick_time_);
    monitor_->publish_tick_duration(duration);
  }
  last_tick_time_ = now;

  if (active_tree_) {
    monitor_->publish_status(*active_tree_);
  }

  if (status == BT::NodeStatus::FAILURE && active_tree_) {
    auto report = failure_handler_->build_report(*active_tree_, status, "Behavior tree failed");
    failure_handler_->request_context_snapshot(report);
    failure_handler_->request_llm_assistance(report);
    monitor_->publish_failure(report);
  }

  return std::nullopt;
}

std::optional<std::string> LLMTreeServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  last_tick_time_.reset();
  active_tree_ = nullptr;
  failure_handler_->reset();

  const auto status_str = BT::toStr(status);
  RCLCPP_INFO(
    logger_, "Tree '%s' completed with status %s (cancelled=%s)", treeName().c_str(),
    status_str.c_str(), was_cancelled ? "true" : "false");

  if (was_cancelled) {
    return std::string("Execution cancelled by client");
  }

  if (status == BT::NodeStatus::FAILURE) {
    return std::string("Behavior tree execution failed");
  }

  return std::nullopt;
}

std::optional<std::string> LLMTreeServer::onLoopFeedback()
{
  if (!publish_feedback_ || !active_tree_) {
    return std::nullopt;
  }

  auto * root = active_tree_->rootNode();
  if (!root) {
    return std::nullopt;
  }

  nlohmann::json feedback{
    {"tree", treeName()},
    {"root_node", root->name()},
    {"status", BT::toStr(root->status())},
  };

  return feedback.dump();
}

}  // namespace bt_executor
