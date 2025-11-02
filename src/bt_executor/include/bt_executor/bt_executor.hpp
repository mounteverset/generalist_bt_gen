#ifndef BT_EXECUTOR__BT_EXECUTOR_HPP_
#define BT_EXECUTOR__BT_EXECUTOR_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace bt_executor
{

class BTLoader;
class BTFailureHandler;
class BTMonitor;

struct ExecutorConfig
{
  std::string tree_xml_path;
  std::string config_yaml_path;
  std::chrono::milliseconds tick_period{100};
  bool auto_start{true};
};

class BTExecutor : public rclcpp::Node
{
public:
  explicit BTExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BTExecutor() override;

  void configure();
  void start();
  void stop();
  void tick();
  bool is_running() const;

  const ExecutorConfig & config() const;

  BT::Tree & tree();
  BT::Blackboard::Ptr blackboard();

  BTLoader & loader();
  BTMonitor & monitor();
  BTFailureHandler & failure_handler();

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr context_request_client() const;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr update_tree_client() const;

private:
  void declare_parameters();
  void read_parameters();
  void initialize_components();
  void initialize_services();
  void create_tree();
  void schedule_tick_timer();
  void teardown_tree();

  ExecutorConfig config_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;

  std::shared_ptr<BTLoader> loader_;
  std::shared_ptr<BTMonitor> monitor_;
  std::shared_ptr<BTFailureHandler> failure_handler_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr context_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr updater_client_;

  std::atomic_bool configured_{false};
  std::atomic_bool running_{false};
};

}  // namespace bt_executor

#endif  // BT_EXECUTOR__BT_EXECUTOR_HPP_
