#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <chrono>
#include <mutex>
#include <string>

namespace robot_actions
{

class SetMavrosMode : public BT::RosServiceNode<mavros_msgs::srv::SetMode>
{
public:
  SetMavrosMode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr & request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  bool enable_debug_logging_{false};
  std::string requested_mode_;
};

class SetMavrosArm : public BT::RosServiceNode<mavros_msgs::srv::CommandBool>
{
public:
  SetMavrosArm(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr & request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  bool requested_arm_{true};
};

class WaitForMavrosMode : public BT::StatefulActionNode
{
public:
  WaitForMavrosMode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  std::mutex mode_mutex_;
  std::string current_mode_;
  std::string desired_mode_;
  bool current_armed_{false};
  bool require_armed_{false};
  std::chrono::steady_clock::time_point deadline_;
};

class WaitForMavrosGpsFix : public BT::StatefulActionNode
{
public:
  WaitForMavrosGpsFix(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_subscription_;
  std::mutex fix_mutex_;
  bool have_fix_{false};
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace robot_actions
