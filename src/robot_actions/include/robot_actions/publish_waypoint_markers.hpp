#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace robot_actions
{

std::string marker_frame_or_map(
  const std::string & requested_frame,
  const std::vector<std::string> & available_frames);

class PublishWaypointMarkers : public BT::StatefulActionNode
{
public:
  PublishWaypointMarkers(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct CartesianWaypoint
  {
    geometry_msgs::msg::Point point;
    double yaw{0.0};
  };

  struct GeographicWaypoint
  {
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
    double yaw{0.0};
  };

  using FromLL = robot_localization::srv::FromLL;

  void read_parameters();
  static bool parse_cartesian_waypoint(
    const std::string & raw,
    CartesianWaypoint & waypoint);
  void parse_map_waypoints(const std::string & raw);
  void parse_gps_waypoints(const std::string & raw);
  bool start_next_gps_conversion();
  BT::NodeStatus finish();
  bool conversion_timed_out() const;
  void clear_pending_request();
  void publish_markers();
  std::string available_frame_or_map(const std::string & requested_frame) const;
  void append_route_markers(
    visualization_msgs::msg::MarkerArray & markers,
    const std::vector<CartesianWaypoint> & waypoints,
    const std::string & frame_id,
    const std::string & route_name,
    float red,
    float green,
    float blue) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Client<FromLL>::SharedPtr from_ll_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  FromLL::Response::SharedPtr pending_response_;
  rclcpp::Client<FromLL>::SharedFuture pending_future_;
  int64_t pending_request_id_{0};

  std::string marker_topic_{"/mission_coordinator/waypoint_markers"};
  std::string from_ll_service_{"/fromLL"};
  std::string default_map_frame_{"target/map"};
  std::string default_gps_frame_{"target/odom"};
  std::string map_frame_;
  std::string gps_frame_;
  int conversion_timeout_ms_{2000};
  double line_width_{0.08};
  double arrow_scale_x_{0.65};
  double arrow_scale_y_{0.18};
  double arrow_scale_z_{0.18};
  double label_height_{0.35};
  bool enable_debug_logging_{false};

  std::vector<CartesianWaypoint> map_waypoints_;
  std::vector<GeographicWaypoint> gps_waypoints_;
  std::vector<CartesianWaypoint> converted_gps_waypoints_;
  std::size_t gps_conversion_index_{0};
  std::chrono::steady_clock::time_point conversion_started_;
};

}  // namespace robot_actions
