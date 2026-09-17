#include "robot_actions/publish_waypoint_markers.hpp"

#include "robot_actions/common.hpp"
#include "robot_actions/gps_waypoint_utils.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <future>
#include <sstream>
#include <string>
#include <utility>

namespace robot_actions
{

namespace
{

std::string trim_copy(const std::string & input)
{
  const auto front = std::find_if_not(
    input.begin(), input.end(), [](unsigned char value) {return std::isspace(value);});
  const auto back = std::find_if_not(
    input.rbegin(), input.rend(), [](unsigned char value) {return std::isspace(value);}).base();
  return front < back ? std::string(front, back) : std::string{};
}

std::vector<std::string> split_entries(const std::string & raw)
{
  std::string cleaned = trim_copy(raw);
  if (cleaned.size() >= 2 && cleaned.front() == '[' && cleaned.back() == ']') {
    cleaned = cleaned.substr(1, cleaned.size() - 2);
  }

  std::vector<std::string> entries;
  std::stringstream stream(cleaned);
  std::string entry;
  while (std::getline(stream, entry, ';')) {
    entry = trim_copy(entry);
    if (!entry.empty()) {
      entries.push_back(entry);
    }
  }
  return entries;
}

bool parse_number(const std::string & raw, double & value)
{
  const std::string cleaned = trim_copy(raw);
  if (cleaned.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0;
    value = std::stod(cleaned, &consumed);
    return consumed == cleaned.size() && std::isfinite(value);
  } catch (const std::exception &) {
    return false;
  }
}

template<typename T>
T parameter_or_declare(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

}  // namespace

PublishWaypointMarkers::PublishWaypointMarkers(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config), node_(params.nh.lock())
{
  if (!node_) {
    return;
  }

  enable_debug_logging_ = is_debug_logging_enabled(node_);
  read_parameters();
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, qos);
  from_ll_client_ = node_->create_client<FromLL>(from_ll_service_);
}

BT::PortsList PublishWaypointMarkers::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "waypoints", "",
      "Semicolon-separated Cartesian waypoints formatted as x,y[,yaw]."),
    BT::InputPort<std::string>(
      "gps_waypoints", "",
      "Semicolon-separated geographic waypoints formatted as lat,lon[,yaw] or "
      "lat,lon,alt,yaw."),
    BT::InputPort<std::string>(
      "waypoint_frame_id", "",
      "Frame for Cartesian waypoints. Defaults to waypoint_marker_map_frame."),
    BT::InputPort<std::string>(
      "gps_frame_id", "",
      "Frame of /fromLL results. Defaults to waypoint_marker_gps_frame.")
  };
}

BT::NodeStatus PublishWaypointMarkers::onStart()
{
  clear_pending_request();
  map_waypoints_.clear();
  gps_waypoints_.clear();
  converted_gps_waypoints_.clear();
  gps_conversion_index_ = 0;

  if (!node_ || !publisher_) {
    RCLCPP_WARN(get_logger(), "PublishWaypointMarkers -> ROS node is unavailable; skipping.");
    return BT::NodeStatus::SUCCESS;
  }

  map_frame_ = trim_copy(getInput<std::string>("waypoint_frame_id").value_or(""));
  if (map_frame_.empty()) {
    map_frame_ = default_map_frame_;
  }
  gps_frame_ = trim_copy(getInput<std::string>("gps_frame_id").value_or(""));
  if (gps_frame_.empty()) {
    gps_frame_ = default_gps_frame_;
  }

  parse_map_waypoints(getInput<std::string>("waypoints").value_or(""));
  parse_gps_waypoints(getInput<std::string>("gps_waypoints").value_or(""));

  if (gps_waypoints_.empty()) {
    return finish();
  }

  conversion_started_ = std::chrono::steady_clock::now();
  if (from_ll_client_ && from_ll_client_->service_is_ready()) {
    start_next_gps_conversion();
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishWaypointMarkers::onRunning()
{
  if (conversion_timed_out()) {
    RCLCPP_WARN(
      get_logger(),
      "PublishWaypointMarkers -> GPS conversion through '%s' timed out; publishing available "
      "markers and continuing the mission.",
      from_ll_service_.c_str());
    clear_pending_request();
    return finish();
  }

  if (!pending_future_.valid()) {
    if (from_ll_client_ && from_ll_client_->service_is_ready()) {
      start_next_gps_conversion();
    }
    return BT::NodeStatus::RUNNING;
  }

  if (pending_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  pending_response_ = pending_future_.get();
  pending_future_ = {};
  pending_request_id_ = 0;
  if (pending_response_) {
    const auto & point = pending_response_->map_point;
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      CartesianWaypoint converted;
      converted.point = point;
      converted.yaw = gps_waypoints_[gps_conversion_index_].yaw;
      converted_gps_waypoints_.push_back(converted);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "PublishWaypointMarkers -> /fromLL returned non-finite coordinates for GPS waypoint %zu.",
        gps_conversion_index_ + 1);
    }
  }

  ++gps_conversion_index_;
  pending_response_.reset();
  if (gps_conversion_index_ >= gps_waypoints_.size()) {
    return finish();
  }

  start_next_gps_conversion();
  return BT::NodeStatus::RUNNING;
}

void PublishWaypointMarkers::onHalted()
{
  clear_pending_request();
  map_waypoints_.clear();
  gps_waypoints_.clear();
  converted_gps_waypoints_.clear();
  gps_conversion_index_ = 0;
}

void PublishWaypointMarkers::read_parameters()
{
  marker_topic_ = parameter_or_declare<std::string>(
    node_, "waypoint_marker_topic", marker_topic_);
  from_ll_service_ = parameter_or_declare<std::string>(
    node_, "waypoint_marker_from_ll_service", from_ll_service_);
  default_map_frame_ = parameter_or_declare<std::string>(
    node_, "waypoint_marker_map_frame", default_map_frame_);
  default_gps_frame_ = parameter_or_declare<std::string>(
    node_, "waypoint_marker_gps_frame", default_gps_frame_);
  conversion_timeout_ms_ = std::max(
    0,
    parameter_or_declare<int>(
      node_, "waypoint_marker_conversion_timeout_ms", conversion_timeout_ms_));
}

bool PublishWaypointMarkers::parse_cartesian_waypoint(
  const std::string & raw,
  CartesianWaypoint & waypoint)
{
  std::string cleaned = raw;
  cleaned.erase(
    std::remove_if(
      cleaned.begin(), cleaned.end(),
      [](unsigned char value) {
        return value == '[' || value == ']' || value == '(' || value == ')';
      }),
    cleaned.end());

  std::stringstream stream(cleaned);
  std::string token;
  std::vector<double> values;
  while (std::getline(stream, token, ',')) {
    double value = 0.0;
    if (!parse_number(token, value)) {
      return false;
    }
    values.push_back(value);
  }
  if (values.size() < 2 || values.size() > 3) {
    return false;
  }

  waypoint.point.x = values[0];
  waypoint.point.y = values[1];
  waypoint.point.z = 0.0;
  waypoint.yaw = values.size() == 3 ? values[2] : 0.0;
  return true;
}

void PublishWaypointMarkers::parse_map_waypoints(const std::string & raw)
{
  const auto entries = split_entries(raw);
  for (std::size_t index = 0; index < entries.size(); ++index) {
    CartesianWaypoint waypoint;
    if (!parse_cartesian_waypoint(entries[index], waypoint)) {
      RCLCPP_WARN(
        get_logger(),
        "PublishWaypointMarkers -> skipping invalid Cartesian waypoint %zu ('%s').",
        index + 1, entries[index].c_str());
      continue;
    }
    map_waypoints_.push_back(waypoint);
  }
}

void PublishWaypointMarkers::parse_gps_waypoints(const std::string & raw)
{
  const auto entries = split_gps_waypoints(raw);
  for (std::size_t index = 0; index < entries.size(); ++index) {
    GpsWaypoint parsed;
    if (!parse_gps_waypoint(entries[index], parsed)) {
      RCLCPP_WARN(
        get_logger(),
        "PublishWaypointMarkers -> skipping invalid GPS waypoint %zu ('%s').",
        index + 1, entries[index].c_str());
      continue;
    }

    GeographicWaypoint waypoint;
    waypoint.latitude = parsed.latitude;
    waypoint.longitude = parsed.longitude;
    waypoint.altitude = parsed.altitude;
    waypoint.yaw = parsed.yaw;
    gps_waypoints_.push_back(waypoint);
  }
}

bool PublishWaypointMarkers::start_next_gps_conversion()
{
  if (!from_ll_client_ || gps_conversion_index_ >= gps_waypoints_.size()) {
    return false;
  }

  const auto & waypoint = gps_waypoints_[gps_conversion_index_];
  auto request = std::make_shared<FromLL::Request>();
  request->ll_point.latitude = waypoint.latitude;
  request->ll_point.longitude = waypoint.longitude;
  request->ll_point.altitude = waypoint.altitude;
  try {
    auto future_and_id = from_ll_client_->async_send_request(request);
    pending_request_id_ = future_and_id.request_id;
    pending_future_ = future_and_id.future.share();
    return true;
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(),
      "PublishWaypointMarkers -> could not request GPS waypoint conversion: %s",
      error.what());
    return false;
  }
}

BT::NodeStatus PublishWaypointMarkers::finish()
{
  publish_markers();
  if (enable_debug_logging_) {
    RCLCPP_INFO(
      get_logger(),
      "PublishWaypointMarkers -> published %zu map and %zu GPS waypoint marker(s) on '%s'.",
      map_waypoints_.size(), converted_gps_waypoints_.size(), marker_topic_.c_str());
  }
  clear_pending_request();
  return BT::NodeStatus::SUCCESS;
}

bool PublishWaypointMarkers::conversion_timed_out() const
{
  return std::chrono::steady_clock::now() - conversion_started_ >=
         std::chrono::milliseconds(conversion_timeout_ms_);
}

void PublishWaypointMarkers::clear_pending_request()
{
  if (from_ll_client_ && pending_request_id_ != 0) {
    from_ll_client_->remove_pending_request(pending_request_id_);
  }
  pending_request_id_ = 0;
  pending_future_ = {};
  pending_response_.reset();
}

void PublishWaypointMarkers::publish_markers()
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  append_route_markers(
    markers, map_waypoints_, map_frame_, "map", 0.10F, 0.75F, 1.00F);
  append_route_markers(
    markers, converted_gps_waypoints_, gps_frame_, "gps", 1.00F, 0.55F, 0.10F);
  publisher_->publish(markers);
}

void PublishWaypointMarkers::append_route_markers(
  visualization_msgs::msg::MarkerArray & markers,
  const std::vector<CartesianWaypoint> & waypoints,
  const std::string & frame_id,
  const std::string & route_name,
  float red,
  float green,
  float blue) const
{
  if (waypoints.empty() || frame_id.empty()) {
    return;
  }

  const auto stamp = node_->get_clock()->now();
  visualization_msgs::msg::Marker line;
  line.header.frame_id = frame_id;
  line.header.stamp = stamp;
  line.ns = "mission_waypoints_" + route_name + "_route";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = 0.08;
  line.color.r = red;
  line.color.g = green;
  line.color.b = blue;
  line.color.a = 0.85F;
  line.frame_locked = true;

  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    auto route_point = waypoints[index].point;
    route_point.z += 0.10;
    line.points.push_back(route_point);

    visualization_msgs::msg::Marker arrow;
    arrow.header = line.header;
    arrow.ns = "mission_waypoints_" + route_name;
    arrow.id = static_cast<int>(index);
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.position = route_point;
    arrow.pose.orientation.z = std::sin(waypoints[index].yaw / 2.0);
    arrow.pose.orientation.w = std::cos(waypoints[index].yaw / 2.0);
    arrow.scale.x = 0.65;
    arrow.scale.y = 0.18;
    arrow.scale.z = 0.18;
    arrow.color.r = red;
    arrow.color.g = green;
    arrow.color.b = blue;
    arrow.color.a = 1.0F;
    arrow.frame_locked = true;
    markers.markers.push_back(std::move(arrow));

    visualization_msgs::msg::Marker label;
    label.header = line.header;
    label.ns = "mission_waypoints_" + route_name + "_labels";
    label.id = static_cast<int>(index);
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position = waypoints[index].point;
    label.pose.position.z += 0.65;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.35;
    label.color.r = red;
    label.color.g = green;
    label.color.b = blue;
    label.color.a = 1.0F;
    label.text = std::to_string(index + 1);
    label.frame_locked = true;
    markers.markers.push_back(std::move(label));
  }

  if (waypoints.size() > 1) {
    markers.markers.push_back(std::move(line));
  }
}

}  // namespace robot_actions
