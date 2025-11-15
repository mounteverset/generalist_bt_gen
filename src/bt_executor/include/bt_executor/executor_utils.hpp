#pragma once

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/decorators/loop_node.h>
#include <nlohmann/json.hpp>
#include <rclcpp/logger.hpp>

namespace bt_executor
{

struct BlackboardLoadStats
{
  std::size_t entries_written{0};
  std::size_t warnings{0};
};

/// Parse a JSON payload string; returns empty object if the payload is blank or invalid.
nlohmann::json parse_payload_json(const std::string & payload, const rclcpp::Logger & logger);

/// Recursively write JSON keys/values onto the behavior-tree blackboard.
BlackboardLoadStats load_payload_into_blackboard(
  const BT::Blackboard::Ptr & blackboard,
  const nlohmann::json & payload,
  const rclcpp::Logger & logger);

}  // namespace bt_executor
