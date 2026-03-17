#pragma once

#include <rclcpp/rclcpp.hpp>

namespace robot_actions
{

inline rclcpp::Logger get_logger()
{
  static rclcpp::Logger logger = rclcpp::get_logger("robot_actions");
  return logger;
}

inline bool is_debug_logging_enabled(const rclcpp::Node::SharedPtr & node)
{
  if (!node) {
    return false;
  }
  if (node->has_parameter("enable_debug_logging")) {
    return node->get_parameter("enable_debug_logging").as_bool();
  }
  return node->declare_parameter<bool>("enable_debug_logging", false);
}

}
