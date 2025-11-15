#include "bt_executor/executor_utils.hpp"

#include <rclcpp/logging.hpp>
#include <string>
#include <utility>

namespace bt_executor
{
namespace
{

void assign_string(
  const BT::Blackboard::Ptr & blackboard, const std::string & key, std::string value,
  BlackboardLoadStats & stats, const rclcpp::Logger & logger)
{
  try {
    blackboard->set(key, std::move(value));
    ++stats.entries_written;
  } catch (const std::exception & e) {
    ++stats.warnings;
    RCLCPP_WARN(logger, "Failed to set blackboard key '%s': %s", key.c_str(), e.what());
  }
}

std::string json_value_to_string(const nlohmann::json & value)
{
  if (value.is_string()) {
    return value.get<std::string>();
  }
  return value.dump();
}

void populate_recursive(
  const BT::Blackboard::Ptr & blackboard, const std::string & key_path,
  const nlohmann::json & value, BlackboardLoadStats & stats, const rclcpp::Logger & logger)
{
  if (!value.is_object()) {
    if (value.is_null()) {
      ++stats.warnings;
      RCLCPP_WARN(logger, "Blackboard key '%s' skipped (null value).", key_path.c_str());
      return;
    }
    assign_string(blackboard, key_path, json_value_to_string(value), stats, logger);
    return;
  }

  assign_string(blackboard, key_path, value.dump(), stats, logger);
  for (const auto & [child_key, child_value] : value.items()) {
    const auto nested_key = key_path.empty() ? child_key : key_path + "." + child_key;
    populate_recursive(blackboard, nested_key, child_value, stats, logger);
  }
}

}  // namespace

nlohmann::json parse_payload_json(const std::string & payload, const rclcpp::Logger & logger)
{
  if (payload.empty()) {
    return nlohmann::json::object();
  }
  try {
    auto data = nlohmann::json::parse(payload);
    if (!data.is_object()) {
      RCLCPP_WARN(logger, "Mission payload is not a JSON object; ignoring.");
      return nlohmann::json::object();
    }
    return data;
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger, "Failed to parse mission payload: %s", e.what());
    return nlohmann::json::object();
  }
}

BlackboardLoadStats load_payload_into_blackboard(
  const BT::Blackboard::Ptr & blackboard, const nlohmann::json & payload,
  const rclcpp::Logger & logger)
{
  BlackboardLoadStats stats;
  if (!payload.is_object()) {
    return stats;
  }
  for (const auto & [key, value] : payload.items()) {
    populate_recursive(blackboard, key, value, stats, logger);
  }
  return stats;
}

}  // namespace bt_executor
