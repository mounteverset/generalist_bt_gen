#ifndef BT_EXECUTOR__BT_MONITOR_HPP_
#define BT_EXECUTOR__BT_MONITOR_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"

#include "bt_executor/bt_failure_handler.hpp"

namespace bt_executor
{

class BTMonitor
{
public:
  explicit BTMonitor(rclcpp::Node & node);

  void publish_status(const BT::Tree & tree);

  void publish_tick_duration(std::chrono::milliseconds duration);

  void publish_failure(const FailureReport & report);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;

  rclcpp::Logger logger_;
  rclcpp::Node & node_;
};
}  // namespace bt_executor

#endif  // BT_EXECUTOR__BT_MONITOR_HPP_
