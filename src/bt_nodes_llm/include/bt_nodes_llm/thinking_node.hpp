#ifndef BT_NODES_LLM__THINKING_NODE_HPP_
#define BT_NODES_LLM__THINKING_NODE_HPP_

#include <future>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "llm_interface/srv/llm_query.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_nodes_llm
{

class ThinkingNode : public BT::StatefulActionNode
{
public:
  using LLMQuery = llm_interface::srv::LLMQuery;
  using LLMQueryFuture = rclcpp::Client<LLMQuery>::SharedFuture;
  using GoalHandle = rclcpp::Client<LLMQuery>;

  explicit ThinkingNode(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr get_node();
  std::string resolve_prompt();
  std::string resolve_context();
  std::string resolve_metadata();
  bool ensure_client(const std::string & service_name, unsigned int wait_timeout_ms);
  void process_success_response(const LLMQuery::Response & response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<LLMQuery>::SharedPtr client_;
  std::string current_service_name_{"/llm_query"};
  rclcpp::Time start_time_{0};
  LLMQueryFuture response_future_;
  unsigned int timeout_ms_{5000U};
  std::string response_key_{"llm_response"};
  std::string decision_key_{"llm_decision"};
};

}  // namespace bt_nodes_llm

#endif  // BT_NODES_LLM__THINKING_NODE_HPP_
