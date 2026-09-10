#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "robot_actions/gps_waypoint_utils.hpp"

#include <chrono>
#include <mutex>
#include <string>

namespace robot_actions
{

class MoveToGlobalSetpoint : public BT::DecoratorNode
{
public:
  MoveToGlobalSetpoint(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  bool start_navigation();
  void stop_navigation();
  void position_callback(sensor_msgs::msg::NavSatFix::ConstSharedPtr message);
  static double distance_m(
    double latitude_a,
    double longitude_a,
    double latitude_b,
    double longitude_b);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr setpoint_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_subscription_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;

  std::mutex position_mutex_;
  double latest_latitude_{0.0};
  double latest_longitude_{0.0};
  bool have_position_fix_{false};
  bool arrived_{false};

  GpsWaypoint target_;
  double acceptance_radius_m_{3.0};
  bool enable_debug_logging_{false};
};

}  // namespace robot_actions
