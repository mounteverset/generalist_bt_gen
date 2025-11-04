#include "bt_nodes_llm/thinking_node.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp/exceptions.h"

namespace bt_nodes_llm
{
namespace
{
constexpr char kBlackboardNodeKey[] = "ros_node";
constexpr char kDefaultCommandKey[] = "user_command";
constexpr char kDefaultContextKey[] = "execution_context";
constexpr char kDefaultMetadataKey[] = "llm_metadata";
constexpr char kDefaultServiceName[] = "/llm_query";
constexpr char kDefaultResponseKey[] = "llm_response";
constexpr char kDefaultDecisionKey[] = "llm_decision";
}

ThinkingNode::ThinkingNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList ThinkingNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("prompt", "Optional explicit prompt override"),
    BT::InputPort<std::string>("command_key", kDefaultCommandKey, "Blackboard key holding the user command"),
    BT::InputPort<std::string>("context_key", kDefaultContextKey, "Blackboard key with JSON execution context"),
    BT::InputPort<std::string>("metadata_key", kDefaultMetadataKey, "Blackboard key with metadata JSON"),
    BT::InputPort<std::string>("service_name", kDefaultServiceName, "LLM query service name"),
    BT::InputPort<unsigned int>("service_wait_timeout_ms", 1000U, "Milliseconds to wait for service availability"),
    BT::InputPort<unsigned int>("timeout_ms", 5000U, "Milliseconds before failing the LLM query"),
    BT::InputPort<std::string>("response_key", kDefaultResponseKey, "Blackboard key to store raw LLM response JSON"),
    BT::InputPort<std::string>("decision_key", kDefaultDecisionKey, "Blackboard key for the decision string"),
    BT::OutputPort<std::string>("decision", "Decision string returned by the LLM")
  };
}

BT::NodeStatus ThinkingNode::onStart()
{
  auto node = get_node();

  timeout_ms_ = getInput<unsigned int>("timeout_ms").value_or(5000U);
  response_key_ = getInput<std::string>("response_key").value_or(kDefaultResponseKey);
  decision_key_ = getInput<std::string>("decision_key").value_or(kDefaultDecisionKey);

  const auto service_name = getInput<std::string>("service_name").value_or(std::string{kDefaultServiceName});
  const auto wait_timeout_ms = getInput<unsigned int>("service_wait_timeout_ms").value_or(1000U);
  if (!ensure_client(service_name, wait_timeout_ms)) {
    RCLCPP_WARN(
      node->get_logger(), "ThinkingNode [%s]: service '%s' unavailable",
      name().c_str(), service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<LLMQuery::Request>();
  request->prompt = resolve_prompt();
  request->context_json = resolve_context();
  request->metadata_json = resolve_metadata();

  start_time_ = node->now();
  auto future_and_id = client_->async_send_request(request);
  response_future_ = future_and_id.share();
  if (!response_future_.valid()) {
    RCLCPP_ERROR(node->get_logger(), "ThinkingNode [%s]: failed to send request", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ThinkingNode::onRunning()
{
  auto node = get_node();
  rclcpp::spin_some(node);
  if (!response_future_.valid()) {
    RCLCPP_ERROR(node->get_logger(), "ThinkingNode [%s]: invalid future", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto status = response_future_.wait_for(std::chrono::milliseconds(0));
  if (status == std::future_status::ready) {
    auto response_ptr = response_future_.get();
    if (!response_ptr) {
      RCLCPP_ERROR(node->get_logger(), "ThinkingNode [%s]: null response received", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    const auto & response = *response_ptr;
    if (!response.success) {
      RCLCPP_WARN(
        node->get_logger(), "ThinkingNode [%s]: LLM reported failure: %s",
        name().c_str(), response.reasoning.c_str());
      return BT::NodeStatus::FAILURE;
    }
    process_success_response(response);
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_ms_ > 0U) {
    const rclcpp::Duration timeout{std::chrono::milliseconds(timeout_ms_)};
    if (node->now() - start_time_ > timeout) {
      RCLCPP_WARN(
        node->get_logger(), "ThinkingNode [%s]: timed out after %u ms",
        name().c_str(), timeout_ms_);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void ThinkingNode::onHalted()
{
  response_future_ = LLMQueryFuture{};
}

rclcpp::Node::SharedPtr ThinkingNode::get_node()
{
  if (!node_) {
    try {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>(kBlackboardNodeKey);
    } catch (const BT::RuntimeError &) {
      throw std::runtime_error(
              "ThinkingNode requires a rclcpp::Node shared pointer stored in blackboard key '" +
              std::string{kBlackboardNodeKey} + "'");
    }
  }
  return node_;
}

std::string ThinkingNode::resolve_prompt()
{
  if (auto prompt = getInput<std::string>("prompt")) {
    return prompt.value();
  }

  std::string command_key = getInput<std::string>("command_key").value_or(kDefaultCommandKey);
  std::string command;
  if (!config().blackboard->get(command_key, command)) {
    throw std::runtime_error(
            "ThinkingNode could not find command on blackboard key '" + command_key + "'");
  }
  return command;
}

std::string ThinkingNode::resolve_context()
{
  std::string context_key = getInput<std::string>("context_key").value_or(kDefaultContextKey);
  std::string context;
  (void)config().blackboard->get(context_key, context);
  return context;
}

std::string ThinkingNode::resolve_metadata()
{
  std::string metadata_key = getInput<std::string>("metadata_key").value_or(kDefaultMetadataKey);
  std::string metadata;
  (void)config().blackboard->get(metadata_key, metadata);
  return metadata;
}

bool ThinkingNode::ensure_client(const std::string & service_name, unsigned int wait_timeout_ms)
{
  auto node = get_node();
  if (!client_ || service_name != current_service_name_) {
    client_ = node->create_client<LLMQuery>(service_name);
    current_service_name_ = service_name;
  }

  const auto wait_duration = std::chrono::milliseconds(wait_timeout_ms);
  if (!client_->wait_for_service(wait_duration)) {
    return false;
  }
  return true;
}

void ThinkingNode::process_success_response(const LLMQuery::Response & response)
{
  auto node = get_node();
  config().blackboard->set(response_key_, response.response_json);

  try {
    const auto json = nlohmann::json::parse(response.response_json);
    std::string decision;
    if (json.contains("decision")) {
      decision = json.at("decision").get<std::string>();
    } else if (json.is_string()) {
      decision = json.get<std::string>();
    } else {
      decision = response.reasoning;
    }
    config().blackboard->set(decision_key_, decision);
    setOutput("decision", decision);
  } catch (const nlohmann::json::exception & ex) {
    RCLCPP_WARN(
      node->get_logger(), "ThinkingNode [%s]: failed to parse LLM JSON: %s",
      name().c_str(), ex.what());
    config().blackboard->set(decision_key_, response.reasoning);
    setOutput("decision", response.reasoning);
  }
}

}  // namespace bt_nodes_llm
