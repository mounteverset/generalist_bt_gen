#include "bt_executor/bt_monitor.hpp"

#include <sstream>

#include "behaviortree_cpp/basic_types.h"
#include "nlohmann/json.hpp"
#include "rclcpp/logging.hpp"

namespace bt_executor
{

namespace
{
constexpr char kStatusTopic[] = "bt_executor/status";
constexpr char kDiagnosticsTopic[] = "bt_executor/diagnostics";
}  // namespace

BTMonitor::BTMonitor(rclcpp::Node & node)
: status_pub_{node.create_publisher<std_msgs::msg::String>(kStatusTopic, 10)},
  diagnostics_pub_{node.create_publisher<std_msgs::msg::String>(kDiagnosticsTopic, 10)},
  logger_{node.get_logger()},
  node_{node}
{
}

void BTMonitor::publish_status(const BT::Tree & tree)
{
  if (!status_pub_) {
    RCLCPP_WARN(logger_, "Status publisher not initialised");
    return;
  }

  std_msgs::msg::String msg;

  if (auto * root = tree.rootNode()) {
    std::ostringstream stream;
    stream << "node=" << root->name() << ";status=" << BT::toStr(root->status());
    msg.data = stream.str();
  } else {
    msg.data = "node=<unset>;status=IDLE";
  }

  status_pub_->publish(msg);
}

void BTMonitor::publish_tick_duration(std::chrono::milliseconds duration)
{
  if (!diagnostics_pub_) {
    RCLCPP_WARN(logger_, "Diagnostics publisher not initialised");
    return;
  }

  std_msgs::msg::String msg;
  msg.data = "tick_duration_ms=" + std::to_string(duration.count());
  diagnostics_pub_->publish(msg);
}

void BTMonitor::publish_failure(const FailureReport & report)
{
  if (!diagnostics_pub_) {
    RCLCPP_WARN(logger_, "Diagnostics publisher not initialised");
    return;
  }

  std_msgs::msg::String msg;
  nlohmann::json payload{
    {"failing_node", report.failing_node},
    {"status", BT::toStr(report.status)},
    {"message", report.failure_message},
    {"context", report.context},
  };
  msg.data = payload.dump();
  diagnostics_pub_->publish(msg);
}

}  // namespace bt_executor
