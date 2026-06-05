#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <string>

namespace robot_actions
{

class DistanceTraveled : public BT::StatefulActionNode
{
public:
  DistanceTraveled(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
  };

  bool read_inputs();
  bool read_next_odom(Pose2D & pose);
  double distance_between(const Pose2D & a, const Pose2D & b) const;
  BT::NodeStatus update_distance();
  void reset_measurement_state();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  bool enable_debug_logging_{false};

  std::string default_odom_topic_{"/odom"};
  std::string odom_topic_;
  double interval_m_{5.0};
  int odom_timeout_ms_{50};

  std::optional<Pose2D> last_pose_;
  double distance_since_success_{0.0};
};

}  // namespace robot_actions
