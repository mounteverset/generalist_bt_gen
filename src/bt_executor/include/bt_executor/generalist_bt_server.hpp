#pragma once

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <behaviortree_ros2/tree_execution_server.hpp>

namespace bt_executor
{

class GeneralistBehaviorTreeServer : public BT::TreeExecutionServer
{
public:
  explicit GeneralistBehaviorTreeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void onTreeCreated(BT::Tree & tree) override;
  void registerNodesIntoFactory(BT::BehaviorTreeFactory & factory) override;
  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;
  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) override;
  std::optional<std::string> onLoopFeedback() override;

private:
  void publish_executor_status(const std::string & text);
  void publish_active_node(const std::string & node_name);

  std::string status_topic_;
  std::string active_node_topic_;
  bool auto_restart_on_failure_{false};
  double feedback_rate_hz_{5.0};
  std::vector<std::string> plugin_directories_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_node_publisher_;

  BT::NodeStatus last_tick_status_{BT::NodeStatus::IDLE};
};

}  // namespace bt_executor
