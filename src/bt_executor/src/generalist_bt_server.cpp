#include "bt_executor/generalist_bt_server.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <filesystem>

namespace bt_executor
{

GeneralistBehaviorTreeServer::GeneralistBehaviorTreeServer(const rclcpp::NodeOptions & options)
: BT::TreeExecutionServer(options)
{
  auto node_handle = node();
  status_topic_ = node_handle->declare_parameter<std::string>("status_topic", "/mission_coordinator/status_text");
  active_node_topic_ = node_handle->declare_parameter<std::string>("active_node_topic", "/mission_coordinator/active_subtree");
  auto_restart_on_failure_ = node_handle->declare_parameter<bool>("auto_restart_on_failure", false);
  feedback_rate_hz_ = node_handle->declare_parameter<double>("feedback_rate_hz", 5.0);
  plugin_directories_ = node_handle->declare_parameter<std::vector<std::string>>(
    "plugin_directories", std::vector<std::string>{});

  status_publisher_ = node_handle->create_publisher<std_msgs::msg::String>(status_topic_, 10);
  active_node_publisher_ = node_handle->create_publisher<std_msgs::msg::String>(active_node_topic_, 10);

  publish_executor_status("GeneralistBehaviorTreeServer initialized.");
}

void GeneralistBehaviorTreeServer::onTreeCreated(BT::Tree & tree)
{
  RCLCPP_INFO(node()->get_logger(), "Tree created with root: %s", tree.rootNode()->name().c_str());
  const auto payload = goalPayload();
  if (!payload.empty()) {
    tree.rootBlackboard()->set("user_command", payload);
    RCLCPP_INFO(node()->get_logger(), "Seeded blackboard with user_command payload.");
  }
}

std::optional<BT::NodeStatus> GeneralistBehaviorTreeServer::onLoopAfterTick(BT::NodeStatus status)
{
  last_tick_status_ = status;

  const auto status_string = "BT status: " + BT::toStr(status, true);
  publish_executor_status(status_string);

  const auto active_name = tree().rootNode() ? tree().rootNode()->name() : std::string("unknown");
  publish_active_node(active_name);

  if (status == BT::NodeStatus::FAILURE && !auto_restart_on_failure_) {
    RCLCPP_WARN(node()->get_logger(), "Tree reported FAILURE. Stopping execution.");
    return status;
  }

  return std::nullopt;
}

std::optional<std::string> GeneralistBehaviorTreeServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  std::string message;
  if (was_cancelled) {
    message = "Mission cancelled by client.";
  } else if (status == BT::NodeStatus::SUCCESS) {
    message = "Mission completed successfully.";
  } else if (status == BT::NodeStatus::FAILURE) {
    message = "Mission failed. Awaiting mission coordinator instructions.";
  } else {
    message = "Mission finished with status: " + BT::toStr(status, true);
  }

  publish_executor_status(message);
  return message;
}

std::optional<std::string> GeneralistBehaviorTreeServer::onLoopFeedback()
{
  std::string feedback = "Status=" + BT::toStr(last_tick_status_, true);
  return feedback;
}

void GeneralistBehaviorTreeServer::publish_executor_status(const std::string & text)
{
  if (status_publisher_) {
    std_msgs::msg::String msg;
    msg.data = text;
    status_publisher_->publish(msg);
  }
  RCLCPP_DEBUG(node()->get_logger(), "%s", text.c_str());
}

void GeneralistBehaviorTreeServer::publish_active_node(const std::string & node_name)
{
  if (active_node_publisher_) {
    std_msgs::msg::String msg;
    msg.data = node_name;
    active_node_publisher_->publish(msg);
  }
}

}  // namespace bt_executor
void bt_executor::GeneralistBehaviorTreeServer::registerNodesIntoFactory(BT::BehaviorTreeFactory & factory)
{
  auto resolve_directory = [](const std::string & entry) -> std::string {
    if (std::filesystem::exists(entry)) {
      return entry;
    }
    const auto pos = entry.find('/');
    if (pos == std::string::npos) {
      return entry;
    }
    try {
      const auto pkg = entry.substr(0, pos);
      const auto sub = entry.substr(pos + 1);
      return ament_index_cpp::get_package_share_directory(pkg) + "/" + sub;
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("bt_executor"), "Failed to resolve plugin path %s: %s", entry.c_str(), e.what());
      return {};
    }
  };

  for (const auto & plugin_path : plugin_directories_) {
    const auto directory = resolve_directory(plugin_path);
    if (directory.empty()) {
      continue;
    }
    RCLCPP_INFO(node()->get_logger(), "Searching plugins in: %s", directory.c_str());
    for (const auto & entry : std::filesystem::directory_iterator(directory)) {
      if (entry.path().extension() == ".so") {
        try {
          factory.registerFromPlugin(entry.path().string());
          RCLCPP_INFO(node()->get_logger(), "Registered plugin: %s", entry.path().filename().c_str());
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            node()->get_logger(), "Failed to load plugin %s: %s",
            entry.path().filename().c_str(), e.what());
        }
      }
    }
  }
}
