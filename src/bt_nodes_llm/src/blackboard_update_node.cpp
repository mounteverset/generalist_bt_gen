#include "bt_nodes_llm/blackboard_update_node.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp/exceptions.h"

namespace bt_nodes_llm
{
namespace
{
constexpr char kBlackboardNodeKey[] = "ros_node";
constexpr char kDefaultPromptKey[] = "llm_update_prompt";
constexpr char kDefaultServiceName[] = "/llm_query";
constexpr char kDefaultTargetNamespace[] = "llm";
constexpr char kDefaultRawResponseKey[] = "llm_raw_response";
}

BlackboardUpdateNode::BlackboardUpdateNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList BlackboardUpdateNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("prompt", "Explicit prompt to send to the LLM"),
    BT::InputPort<std::string>("prompt_key", kDefaultPromptKey, "Blackboard key containing the prompt"),
    BT::InputPort<std::string>("service_name", kDefaultServiceName, "LLM query service name"),
    BT::InputPort<unsigned int>("service_wait_timeout_ms", 1000U, "Milliseconds to wait for service availability"),
    BT::InputPort<unsigned int>("timeout_ms", 5000U, "Milliseconds before failing the LLM query"),
    BT::InputPort<std::string>("target_namespace", kDefaultTargetNamespace, "Prefix applied to stored keys"),
    BT::InputPort<std::string>("raw_response_key", kDefaultRawResponseKey, "Blackboard key storing raw JSON"),
    BT::OutputPort<std::string>("raw_response", "Raw JSON returned by the LLM"),
    BT::OutputPort<std::string>("updated_keys", "Comma separated list of updated blackboard keys")
  };
}

BT::NodeStatus BlackboardUpdateNode::onStart()
{
  auto node = get_node();

  timeout_ms_ = getInput<unsigned int>("timeout_ms").value_or(5000U);
  target_namespace_ = getInput<std::string>("target_namespace").value_or(kDefaultTargetNamespace);
  raw_response_key_ = getInput<std::string>("raw_response_key").value_or(kDefaultRawResponseKey);
  updated_keys_.clear();

  const auto service_name = getInput<std::string>("service_name").value_or(std::string{kDefaultServiceName});
  const auto wait_timeout_ms = getInput<unsigned int>("service_wait_timeout_ms").value_or(1000U);
  if (!ensure_client(service_name, wait_timeout_ms)) {
    RCLCPP_WARN(
      node->get_logger(), "BlackboardUpdateNode [%s]: service '%s' unavailable",
      name().c_str(), service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<LLMQuery::Request>();
  request->prompt = resolve_prompt();
  request->context_json.clear();
  request->metadata_json.clear();

  start_time_ = node->now();
  auto future_and_id = client_->async_send_request(request);
  response_future_ = future_and_id.share();
  if (!response_future_.valid()) {
    RCLCPP_ERROR(node->get_logger(), "BlackboardUpdateNode [%s]: failed to send request", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BlackboardUpdateNode::onRunning()
{
  auto node = get_node();
  rclcpp::spin_some(node);
  if (!response_future_.valid()) {
    RCLCPP_ERROR(node->get_logger(), "BlackboardUpdateNode [%s]: invalid future", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto status = response_future_.wait_for(std::chrono::milliseconds(0));
  if (status == std::future_status::ready) {
    auto response_ptr = response_future_.get();
    if (!response_ptr) {
      RCLCPP_ERROR(
        node->get_logger(), "BlackboardUpdateNode [%s]: null response received", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    const auto & response = *response_ptr;
    if (!response.success) {
      RCLCPP_WARN(
        node->get_logger(), "BlackboardUpdateNode [%s]: LLM reported failure: %s",
        name().c_str(), response.reasoning.c_str());
      return BT::NodeStatus::FAILURE;
    }

    config().blackboard->set(raw_response_key_, response.response_json);
    setOutput("raw_response", response.response_json);

    try {
      const auto json = nlohmann::json::parse(response.response_json);
      apply_updates(json);
    } catch (const nlohmann::json::exception & ex) {
      RCLCPP_WARN(
        node->get_logger(), "BlackboardUpdateNode [%s]: failed to parse LLM JSON: %s",
        name().c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }

    std::ostringstream stream;
    for (size_t i = 0; i < updated_keys_.size(); ++i) {
      if (i > 0) {
        stream << ",";
      }
      stream << updated_keys_[i];
    }
    setOutput("updated_keys", stream.str());

    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_ms_ > 0U) {
    const rclcpp::Duration timeout{std::chrono::milliseconds(timeout_ms_)};
    if (node->now() - start_time_ > timeout) {
      RCLCPP_WARN(
        node->get_logger(), "BlackboardUpdateNode [%s]: timed out after %u ms",
        name().c_str(), timeout_ms_);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void BlackboardUpdateNode::onHalted()
{
  response_future_ = LLMQueryFuture{};
  updated_keys_.clear();
}

rclcpp::Node::SharedPtr BlackboardUpdateNode::get_node()
{
  if (!node_) {
    try {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>(kBlackboardNodeKey);
    } catch (const BT::RuntimeError &) {
      throw std::runtime_error(
              "BlackboardUpdateNode requires a rclcpp::Node shared pointer stored in blackboard key '" +
              std::string{kBlackboardNodeKey} + "'");
    }
  }
  return node_;
}

bool BlackboardUpdateNode::ensure_client(const std::string & service_name, unsigned int wait_timeout_ms)
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

std::string BlackboardUpdateNode::resolve_prompt()
{
  if (auto prompt = getInput<std::string>("prompt")) {
    return prompt.value();
  }

  std::string prompt_key = getInput<std::string>("prompt_key").value_or(kDefaultPromptKey);
  std::string prompt_value;
  if (!config().blackboard->get(prompt_key, prompt_value)) {
    throw std::runtime_error(
            "BlackboardUpdateNode could not find prompt on blackboard key '" + prompt_key + "'");
  }
  return prompt_value;
}

void BlackboardUpdateNode::apply_updates(const nlohmann::json & data)
{
  updated_keys_.clear();
  auto node = get_node();

  const auto store_value = [&](const std::string & key, const nlohmann::json & value) {
      const std::string full_key = target_namespace_.empty() ? key : target_namespace_ + "/" + key;
      try {
        if (value.is_string()) {
          config().blackboard->set(full_key, value.get<std::string>());
        } else if (value.is_boolean()) {
          config().blackboard->set(full_key, value.get<bool>());
        } else if (value.is_number_integer()) {
          config().blackboard->set(full_key, value.get<int64_t>());
        } else if (value.is_number_unsigned()) {
          config().blackboard->set(full_key, value.get<uint64_t>());
        } else if (value.is_number_float()) {
          config().blackboard->set(full_key, value.get<double>());
        } else {
          config().blackboard->set(full_key, value.dump());
        }
        updated_keys_.push_back(full_key);
      } catch (const BT::RuntimeError & ex) {
        RCLCPP_WARN(node->get_logger(), "Failed to set blackboard key '%s': %s", full_key.c_str(), ex.what());
      }
    };

  if (data.is_object()) {
    for (const auto & item : data.items()) {
      store_value(item.key(), item.value());
    }
  } else {
    store_value(target_namespace_, data);
  }
}

}  // namespace bt_nodes_llm
