#include "bt_executor/bt_failure_handler.hpp"

#include <chrono>
#include <future>

#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/logging.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace bt_executor
{

namespace
{
constexpr char kContextServiceName[] = "context_gatherer/snapshot";
}  // namespace

BTFailureHandler::BTFailureHandler(rclcpp::Node & node)
: logger_{node.get_logger()}, node_{node}
{
}

void BTFailureHandler::reset()
{
  RCLCPP_DEBUG(logger_, "Failure handler state reset");
}

FailureReport BTFailureHandler::build_report(
  BT::Tree & tree, BT::NodeStatus status, const std::string & failure_message) const
{
  FailureReport report;
  report.failure_message = failure_message;
  report.status = status;

  if (auto * root = tree.rootNode()) {
    report.failing_node = root->name();
    if (root->status() != BT::NodeStatus::IDLE) {
      report.status = root->status();
    }
  }

  if (auto blackboard = tree.rootBlackboard()) {
    report.context["blackboard"] = BT::ExportBlackboardToJSON(*blackboard);
  }

  return report;
}

void BTFailureHandler::request_context_snapshot(FailureReport & report)
{
  auto client = node_.create_client<std_srvs::srv::Trigger>(kContextServiceName);

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(
      logger_, "Context service '%s' unavailable, skipping snapshot", kContextServiceName);
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_WARN(logger_, "Timeout waiting for context snapshot response");
    return;
  }

  auto response = future.get();
  report.context["context_snapshot"]["success"] = response->success;
  report.context["context_snapshot"]["message"] = response->message;
}

void BTFailureHandler::request_llm_assistance(const FailureReport & report)
{
  RCLCPP_INFO(
    logger_, "Requesting LLM assistance for node '%s': %s", report.failing_node.c_str(),
    report.failure_message.c_str());
  // The actual LLM service interaction will be defined once the llm_interface package exists.
}

}  // namespace bt_executor
