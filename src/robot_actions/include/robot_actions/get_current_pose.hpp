#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace robot_actions
{

class GetCurrentPose : public BT::SyncActionNode
{
public:
  GetCurrentPose(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr make_wait_node() const;
  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);
  static double yaw_from_odom(const nav_msgs::msg::Odometry & odom);
  static double yaw_from_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);
  static std::string pose_string(double x, double y, double yaw);

  rclcpp::Node::SharedPtr node_;
  bool enable_debug_logging_{false};
  std::string default_pose_topic_;
  std::string default_odom_topic_{"/odom"};
};

}  // namespace robot_actions
