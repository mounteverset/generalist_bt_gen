#pragma once

#include <rclcpp/rclcpp.hpp>

namespace robot_actions
{

inline rclcpp::Logger get_logger()
{
  static rclcpp::Logger logger = rclcpp::get_logger("robot_actions");
  return logger;
}

}
