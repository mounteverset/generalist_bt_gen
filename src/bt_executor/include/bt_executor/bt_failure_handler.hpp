#ifndef BT_EXECUTOR__BT_FAILURE_HANDLER_HPP_
#define BT_EXECUTOR__BT_FAILURE_HANDLER_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nlohmann/json.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace bt_executor
{

struct FailureReport
{
  std::string failing_node;
  std::string failure_message;
  BT::NodeStatus status{BT::NodeStatus::IDLE};
  nlohmann::json context;
};

class BTFailureHandler
{
public:
  explicit BTFailureHandler(rclcpp::Node & node);

  void reset();

  FailureReport build_report(
    BT::Tree & tree, BT::NodeStatus status, const std::string & failure_message) const;

  void request_context_snapshot(FailureReport & report);

  void request_llm_assistance(const FailureReport & report);

private:
  rclcpp::Logger logger_;
  rclcpp::Node & node_;
};

}  // namespace bt_executor

#endif  // BT_EXECUTOR__BT_FAILURE_HANDLER_HPP_
