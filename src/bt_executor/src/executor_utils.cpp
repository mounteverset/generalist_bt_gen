#include "bt_executor/executor_utils.hpp"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace bt_executor
{
namespace
{

template<typename T>
void assign_value(
  const BT::Blackboard::Ptr & blackboard, const std::string & key, const T & value,
  BlackboardLoadStats & stats, const rclcpp::Logger & logger)
{
  try {
    blackboard->set<T>(key, value);
    ++stats.entries_written;
  } catch (const std::exception & e) {
    ++stats.warnings;
    RCLCPP_WARN(logger, "Failed to set blackboard key '%s': %s", key.c_str(), e.what());
  }
}

bool is_integer(const nlohmann::json & value)
{
  return value.is_number_integer() || value.is_number_unsigned();
}

void store_array(
  const BT::Blackboard::Ptr & blackboard, const std::string & key, const nlohmann::json & array,
  BlackboardLoadStats & stats, const rclcpp::Logger & logger)
{
  if (!array.is_array()) {
    return;
  }
  if (array.empty()) {
    assign_value<std::vector<std::string>>(blackboard, key, {}, stats, logger);
    return;
  }

  auto all_of = [&array](const std::function<bool(const nlohmann::json &)> & predicate) {
    return std::all_of(array.begin(), array.end(), predicate);
  };

  if (all_of([](const nlohmann::json & item) { return item.is_boolean(); })) {
    std::vector<bool> data;
    data.reserve(array.size());
    for (const auto & item : array) {
      data.push_back(item.get<bool>());
    }
    assign_value(blackboard, key, data, stats, logger);
    return;
  }

  if (all_of(is_integer)) {
    std::vector<std::int64_t> data;
    data.reserve(array.size());
    for (const auto & item : array) {
      data.push_back(item.get<std::int64_t>());
    }
    assign_value(blackboard, key, data, stats, logger);
    return;
  }

  if (all_of([](const nlohmann::json & item) { return item.is_number(); })) {
    std::vector<double> data;
    data.reserve(array.size());
    for (const auto & item : array) {
      data.push_back(item.get<double>());
    }
    assign_value(blackboard, key, data, stats, logger);
    return;
  }

  if (all_of([](const nlohmann::json & item) { return item.is_string(); })) {
    std::vector<std::string> data;
    data.reserve(array.size());
    for (const auto & item : array) {
      data.push_back(item.get<std::string>());
    }
    assign_value(blackboard, key, data, stats, logger);
    return;
  }

  std::vector<std::string> serialized;
  serialized.reserve(array.size());
  for (const auto & item : array) {
    serialized.push_back(item.dump());
  }
  assign_value(blackboard, key, serialized, stats, logger);
}

void populate_recursive(
  const BT::Blackboard::Ptr & blackboard, const std::string & key_path,
  const nlohmann::json & value, BlackboardLoadStats & stats, const rclcpp::Logger & logger)
{
  if (value.is_object()) {
    assign_value(blackboard, key_path, value.dump(), stats, logger);
    for (const auto & [child_key, child_value] : value.items()) {
      const auto nested_key = key_path.empty() ? child_key : key_path + "." + child_key;
      populate_recursive(blackboard, nested_key, child_value, stats, logger);
    }
    return;
  }

  if (value.is_array()) {
    store_array(blackboard, key_path, value, stats, logger);
    return;
  }

  if (value.is_null()) {
    ++stats.warnings;
    RCLCPP_WARN(logger, "Blackboard key '%s' skipped (null value).", key_path.c_str());
    return;
  }

  if (value.is_boolean()) {
    assign_value(blackboard, key_path, value.get<bool>(), stats, logger);
    return;
  }

  if (is_integer(value)) {
    assign_value(blackboard, key_path, value.get<std::int64_t>(), stats, logger);
    return;
  }

  if (value.is_number_float()) {
    assign_value(blackboard, key_path, value.get<double>(), stats, logger);
    return;
  }

  if (value.is_string()) {
    assign_value(blackboard, key_path, value.get<std::string>(), stats, logger);
    return;
  }

  // fallback: serialize unknown types
  assign_value(blackboard, key_path, value.dump(), stats, logger);
}

std::string format_pose(double x, double y, double yaw)
{
  std::ostringstream oss;
  oss << x << "," << y << "," << yaw;
  return oss.str();
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

std::optional<std::string> waypoint_entry_to_string(
  const nlohmann::json & entry, const rclcpp::Logger & logger)
{
  try {
    if (entry.is_string()) {
      return entry.get<std::string>();
    }

    if (entry.is_object()) {
      const double x = entry.value("x", 0.0);
      const double y = entry.value("y", 0.0);
      const double yaw = entry.value("yaw", 0.0);
      return format_pose(x, y, yaw);
    }

    if (entry.is_array() && entry.size() >= 3) {
      const double x = entry.at(0).get<double>();
      const double y = entry.at(1).get<double>();
      const double yaw = entry.at(2).get<double>();
      return format_pose(x, y, yaw);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger, "Invalid waypoint entry: %s", e.what());
    return std::nullopt;
  }

  RCLCPP_WARN(logger, "Unsupported waypoint format: %s", entry.dump().c_str());
  return std::nullopt;
}

BT::SharedQueue<std::string> build_waypoint_queue_from_payload(
  const nlohmann::json & payload, const rclcpp::Logger & logger, const std::string & key)
{
  auto queue = std::make_shared<std::deque<std::string>>();
  if (!payload.is_object()) {
    return queue;
  }

  if (!payload.contains(key)) {
    return queue;
  }

  const auto & waypoint_node = payload.at(key);
  if (waypoint_node.is_string()) {
    queue->push_back(waypoint_node.get<std::string>());
    return queue;
  }

  if (!waypoint_node.is_array()) {
    RCLCPP_WARN(logger, "Waypoints field '%s' is not an array; skipping.", key.c_str());
    return queue;
  }

  for (const auto & entry : waypoint_node) {
    if (const auto value = waypoint_entry_to_string(entry, logger)) {
      queue->push_back(*value);
    }
  }
  return queue;
}

}  // namespace bt_executor
