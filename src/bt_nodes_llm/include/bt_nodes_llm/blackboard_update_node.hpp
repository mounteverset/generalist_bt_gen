#ifndef BT_NODES_LLM__BLACKBOARD_UPDATE_NODE_HPP_
#define BT_NODES_LLM__BLACKBOARD_UPDATE_NODE_HPP_

#include <future>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "llm_interface/srv/llm_query.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_nodes_llm
{

class BlackboardUpdateNode : public BT::StatefulActionNode
{
public:
  using LLMQuery = llm_interface::srv::LLMQuery;
  using LLMQueryFuture = rclcpp::Client<LLMQuery>::SharedFuture;

  explicit BlackboardUpdateNode(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr get_node();
  bool ensure_client(const std::string & service_name, unsigned int wait_timeout_ms);
  std::string resolve_prompt();
  void apply_updates(const nlohmann::json & data);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<LLMQuery>::SharedPtr client_;
  std::string current_service_name_{"/llm_query"};
  rclcpp::Time start_time_{0};
  LLMQueryFuture response_future_;
  unsigned int timeout_ms_{5000U};
  std::string target_namespace_{"llm"};
  std::string raw_response_key_{"llm_raw_response"};
  std::vector<std::string> updated_keys_;
};

}  // namespace bt_nodes_llm

#endif  // BT_NODES_LLM__BLACKBOARD_UPDATE_NODE_HPP_
