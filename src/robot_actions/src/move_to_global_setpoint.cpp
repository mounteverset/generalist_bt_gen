#include "robot_actions/move_to_global_setpoint.hpp"

#include "robot_actions/common.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_actions
{

namespace
{

constexpr double kEarthRadiusM = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

}  // namespace

MoveToGlobalSetpoint::MoveToGlobalSetpoint(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::DecoratorNode(name, config), node_(params.nh.lock())
{
  if (!node_) {
    throw std::runtime_error("MoveToGlobalSetpoint requires a ROS node");
  }
  enable_debug_logging_ = is_debug_logging_enabled(node_);
}

BT::PortsList MoveToGlobalSetpoint::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "gps_pose", "Target GPS pose as lat,lon[,yaw] or lat,lon,alt,yaw"),
    BT::InputPort<double>(
      "acceptance_radius_m", 3.0,
        "Distance from the global position at which the target is reached"),
    BT::InputPort<double>("publish_rate_hz", 5.0, "Global setpoint publication rate"),
    BT::InputPort<std::string>(
      "setpoint_topic", "/mavros/setpoint_position/global", "MAVROS global setpoint topic"),
    BT::InputPort<std::string>(
      "position_topic", "/mavros/global_position/global", "MAVROS global position topic")
  };
}

bool MoveToGlobalSetpoint::start_navigation()
{
  const auto raw_pose = getInput<std::string>("gps_pose");
  if (!raw_pose) {
    RCLCPP_ERROR(
      node_->get_logger(), "MoveToGlobalSetpoint -> missing gps_pose: %s",
      raw_pose.error().c_str());
    return false;
  }

  GpsWaypoint waypoint;
  if (!parse_gps_waypoint(*raw_pose, waypoint)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "MoveToGlobalSetpoint -> invalid GPS pose '%s'; expected lat,lon[,yaw] or lat,lon,alt,yaw",
      raw_pose->c_str());
    return false;
  }

  const auto radius = getInput<double>("acceptance_radius_m");
  const auto publish_rate = getInput<double>("publish_rate_hz");
  if (
    !radius || !std::isfinite(*radius) || *radius <= 0.0 || !publish_rate ||
    !std::isfinite(*publish_rate) || *publish_rate <= 0.0)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "MoveToGlobalSetpoint -> acceptance_radius_m and publish_rate_hz must be positive");
    return false;
  }

  const auto setpoint_topic = getInput<std::string>("setpoint_topic").value_or(
    "/mavros/setpoint_position/global");
  const auto position_topic = getInput<std::string>("position_topic").value_or(
    "/mavros/global_position/global");
  if (setpoint_topic.empty() || position_topic.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "MoveToGlobalSetpoint -> topic names must not be empty");
    return false;
  }

  target_ = waypoint;
  acceptance_radius_m_ = *radius;
  arrived_ = false;
  {
    std::lock_guard<std::mutex> lock(position_mutex_);
    latest_latitude_ = 0.0;
    latest_longitude_ = 0.0;
    have_position_fix_ = false;
  }

  setpoint_publisher_ = node_->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
    setpoint_topic, rclcpp::QoS(10).reliable());
  position_subscription_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    position_topic,
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr message) {
      position_callback(message);
    });

  const auto publish = [
    node = node_, publisher = setpoint_publisher_, target = target_]() {
      geographic_msgs::msg::GeoPoseStamped message;
      message.header.stamp = node->now();
      message.header.frame_id = "wgs84";
      message.pose.position.latitude = target.latitude;
      message.pose.position.longitude = target.longitude;
      message.pose.position.altitude = target.altitude;
      message.pose.orientation.z = std::sin(target.yaw / 2.0);
      message.pose.orientation.w = std::cos(target.yaw / 2.0);
      publisher->publish(message);
    };
  publish();
  setpoint_timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / *publish_rate)),
    publish);
  if (enable_debug_logging_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "MoveToGlobalSetpoint -> target lat=%.8f lon=%.8f alt=%.2f radius=%.2f m",
      target_.latitude,
      target_.longitude,
      target_.altitude,
      acceptance_radius_m_);
  }
  return true;
}

BT::NodeStatus MoveToGlobalSetpoint::tick()
{
  if (status() == BT::NodeStatus::IDLE && !start_navigation()) {
    return BT::NodeStatus::FAILURE;
  }

  if (!arrived_) {
    double latitude = 0.0;
    double longitude = 0.0;
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      if (!have_position_fix_) {
        return BT::NodeStatus::RUNNING;
      }
      latitude = latest_latitude_;
      longitude = latest_longitude_;
    }

    const auto distance = distance_m(latitude, longitude, target_.latitude, target_.longitude);
    if (distance > acceptance_radius_m_) {
      return BT::NodeStatus::RUNNING;
    }
    arrived_ = true;
    RCLCPP_INFO(
      node_->get_logger(), "MoveToGlobalSetpoint -> target reached (%.2f m away)", distance);
  }

  const auto child_status = child()->executeTick();
  if (BT::isStatusCompleted(child_status)) {
    stop_navigation();
  }
  return child_status;
}

void MoveToGlobalSetpoint::halt()
{
  stop_navigation();
  BT::DecoratorNode::halt();
  RCLCPP_WARN(node_->get_logger(), "MoveToGlobalSetpoint -> halt requested");
}

void MoveToGlobalSetpoint::stop_navigation()
{
  setpoint_timer_.reset();
  position_subscription_.reset();
  setpoint_publisher_.reset();
  arrived_ = false;
}

void MoveToGlobalSetpoint::position_callback(
  sensor_msgs::msg::NavSatFix::ConstSharedPtr message)
{
  if (
    !message || !std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
    message->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
    message->latitude == 0.0 || message->longitude == 0.0)
  {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(position_mutex_);
    latest_latitude_ = message->latitude;
    latest_longitude_ = message->longitude;
    have_position_fix_ = true;
  }
  emitWakeUpSignal();
}

double MoveToGlobalSetpoint::distance_m(
  double latitude_a,
  double longitude_a,
  double latitude_b,
  double longitude_b)
{
  const auto latitude_delta = radians(latitude_b - latitude_a);
  const auto longitude_delta = radians(longitude_b - longitude_a);
  const auto latitude_a_radians = radians(latitude_a);
  const auto latitude_b_radians = radians(latitude_b);
  const auto sine_latitude = std::sin(latitude_delta / 2.0);
  const auto sine_longitude = std::sin(longitude_delta / 2.0);
  const auto haversine =
    sine_latitude * sine_latitude +
    std::cos(latitude_a_radians) * std::cos(latitude_b_radians) *
    sine_longitude * sine_longitude;
  return kEarthRadiusM * 2.0 * std::atan2(std::sqrt(haversine), std::sqrt(1.0 - haversine));
}

}  // namespace robot_actions
