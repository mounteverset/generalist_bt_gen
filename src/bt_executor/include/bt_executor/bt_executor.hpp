#ifndef BT_EXECUTOR__BT_EXECUTOR_HPP_
#define BT_EXECUTOR__BT_EXECUTOR_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_ros2/tree_execution_server.hpp"

#include "bt_executor/bt_failure_handler.hpp"
#include "bt_executor/bt_monitor.hpp"

namespace bt_executor
{

class LLMTreeServer : public BT::TreeExecutionServer
{
public:
  explicit LLMTreeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void onTreeCreated(BT::Tree & tree) override;
  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;
  std::optional<std::string> onTreeExecutionCompleted(
    BT::NodeStatus status, bool was_cancelled) override;
  std::optional<std::string> onLoopFeedback() override;

private:
  void update_blackboard(BT::Tree & tree);

  rclcpp::Logger logger_;
  std::unique_ptr<BTMonitor> monitor_;
  std::unique_ptr<BTFailureHandler> failure_handler_;

  BT::Tree * active_tree_{nullptr};
  std::optional<std::chrono::steady_clock::time_point> last_tick_time_;

  bool publish_feedback_{true};
};

}  // namespace bt_executor

#endif  // BT_EXECUTOR__BT_EXECUTOR_HPP_
