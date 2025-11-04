#include "context_gatherer/context_gatherer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace context_gatherer
{
namespace
{
constexpr char kDefaultCameraTopic[] = "/camera/image_raw";
constexpr char kDefaultOdometryTopic[] = "/odometry/global";
constexpr char kDefaultMapTopic[] = "/map";
constexpr char kDefaultGeoTopic[] = "/gps/geo";
constexpr char kDefaultTransportHint[] = "raw";
constexpr double kDefaultDataTimeoutSec = 5.0;
constexpr int kDefaultJpegQuality = 70;
constexpr int kDefaultPreviewWidth = 320;
constexpr int kDefaultPreviewHeight = 0;
}  // namespace

ContextGathererNode::ContextGathererNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("context_gatherer", options)
{
  declare_and_load_parameters();
  setup_interfaces();
}

void ContextGathererNode::declare_and_load_parameters()
{
  camera_topic_ = declare_parameter<std::string>("camera_topic", kDefaultCameraTopic);
  odometry_topic_ = declare_parameter<std::string>("odometry_topic", kDefaultOdometryTopic);
  map_topic_ = declare_parameter<std::string>("map_topic", kDefaultMapTopic);
  geo_topic_ = declare_parameter<std::string>("geo_topic", kDefaultGeoTopic);
  transport_hint_ = declare_parameter<std::string>("image_transport", kDefaultTransportHint);
  data_timeout_sec_ = declare_parameter<double>("data_timeout_sec", kDefaultDataTimeoutSec);
  const int declared_quality =
    declare_parameter<int>("image_jpeg_quality", kDefaultJpegQuality);
  image_jpeg_quality_ = std::max(1, std::min(100, declared_quality));
  image_target_width_ = declare_parameter<int>("image_target_width", kDefaultPreviewWidth);
  image_target_height_ = declare_parameter<int>("image_target_height", kDefaultPreviewHeight);
}

void ContextGathererNode::setup_interfaces()
{
  using std::placeholders::_1;

  image_subscription_ = image_transport::create_subscription(
    this, camera_topic_, std::bind(&ContextGathererNode::handle_image, this, _1), transport_hint_);

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&ContextGathererNode::handle_odometry, this, _1));

  map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    std::bind(&ContextGathererNode::handle_map, this, _1));

  geo_subscription_ = create_subscription<geographic_msgs::msg::GeoPointStamped>(
    geo_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
    std::bind(&ContextGathererNode::handle_geopoint, this, _1));

  service_ = create_service<std_srvs::srv::Trigger>(
    "get_context",
    std::bind(&ContextGathererNode::handle_get_context, this, std::placeholders::_1, std::placeholders::_2));
}

void ContextGathererNode::handle_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_image_.message = msg;
}

void ContextGathererNode::handle_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_odometry_.message = msg;
}

void ContextGathererNode::handle_map(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_map_.message = msg;
}

void ContextGathererNode::handle_geopoint(const geographic_msgs::msg::GeoPointStamped::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_geo_point_.message = msg;
}

void ContextGathererNode::handle_get_context(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const auto payload = compose_context_payload();
  if (payload.empty()) {
    response->success = false;
    response->message = "No recent context data available";
    return;
  }

  response->success = true;
  response->message = payload.dump();
}

nlohmann::json ContextGathererNode::compose_context_payload() const
{
  sensor_msgs::msg::Image::ConstSharedPtr image_msg;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg;
  geographic_msgs::msg::GeoPointStamped::ConstSharedPtr geo_msg;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    image_msg = latest_image_.message;
    odometry_msg = latest_odometry_.message;
    map_msg = latest_map_.message;
    geo_msg = latest_geo_point_.message;
  }

  const auto now_time = this->get_clock()->now();
  nlohmann::json payload = nlohmann::json::object();
  std::vector<std::string> available_sources;
  std::vector<std::string> missing_sources;

  if (image_msg && !is_stale(image_msg->header.stamp, now_time)) {
    payload["camera"] = compose_camera_payload(image_msg);
    available_sources.emplace_back("camera");
  } else {
    missing_sources.emplace_back("camera");
  }

  if (odometry_msg && !is_stale(odometry_msg->header.stamp, now_time)) {
    payload["odometry"] = compose_odometry_payload(odometry_msg);
    available_sources.emplace_back("odometry");
  } else {
    missing_sources.emplace_back("odometry");
  }

  if (map_msg && !is_stale(map_msg->header.stamp, now_time)) {
    payload["map"] = compose_map_payload(map_msg);
    available_sources.emplace_back("map");
  } else {
    missing_sources.emplace_back("map");
  }

  if (geo_msg && !is_stale(geo_msg->header.stamp, now_time)) {
    payload["geolocation"] = compose_geo_payload(geo_msg);
    available_sources.emplace_back("geolocation");
  } else {
    missing_sources.emplace_back("geolocation");
  }

  payload["metadata"] = {
    {"available_sources", available_sources},
    {"missing_sources", missing_sources},
    {"generated_at", {
        {"sec", now_time.seconds()},
        {"nanosec", now_time.nanoseconds()}
      }},
    {"data_timeout_sec", data_timeout_sec_}
  };

  // Remove metadata only case to simplify detection of empty payload.
  if (available_sources.empty()) {
    payload.clear();
  }

  return payload;
}

nlohmann::json ContextGathererNode::compose_camera_payload(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
{
  nlohmann::json json_payload = {
    {"frame_id", msg->header.frame_id},
    {"stamp", {
        {"sec", msg->header.stamp.sec},
        {"nanosec", msg->header.stamp.nanosec}
      }},
    {"encoding", msg->encoding},
    {"height", msg->height},
    {"width", msg->width}
  };

  const auto preview = encode_image_preview(*msg);
  if (!preview.empty()) {
    json_payload["preview"] = {
      {"encoding", "jpeg"},
      {"data_base64", preview},
      {"quality", image_jpeg_quality_},
      {"target_width", image_target_width_},
      {"target_height", image_target_height_}
    };
  }

  return json_payload;
}

nlohmann::json ContextGathererNode::compose_odometry_payload(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg) const
{
  const auto & pose = msg->pose.pose;
  const auto & twist = msg->twist.twist;

  return {
    {"frame_id", msg->header.frame_id},
    {"stamp", {
        {"sec", msg->header.stamp.sec},
        {"nanosec", msg->header.stamp.nanosec}
      }},
    {"pose", {
        {"position", {
            {"x", pose.position.x},
            {"y", pose.position.y},
            {"z", pose.position.z}
          }},
        {"orientation", {
            {"x", pose.orientation.x},
            {"y", pose.orientation.y},
            {"z", pose.orientation.z},
            {"w", pose.orientation.w}
          }}
      }},
    {"twist", {
        {"linear", {
            {"x", twist.linear.x},
            {"y", twist.linear.y},
            {"z", twist.linear.z}
          }},
        {"angular", {
            {"x", twist.angular.x},
            {"y", twist.angular.y},
            {"z", twist.angular.z}
          }}
      }}
  };
}

nlohmann::json ContextGathererNode::compose_map_payload(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg) const
{
  const auto ratio = compute_occupancy_ratio(*msg);

  return {
    {"frame_id", msg->header.frame_id},
    {"stamp", {
        {"sec", msg->header.stamp.sec},
        {"nanosec", msg->header.stamp.nanosec}
      }},
    {"resolution", msg->info.resolution},
    {"width", msg->info.width},
    {"height", msg->info.height},
    {"origin", {
        {"position", {
            {"x", msg->info.origin.position.x},
            {"y", msg->info.origin.position.y},
            {"z", msg->info.origin.position.z}
          }},
        {"orientation", {
            {"x", msg->info.origin.orientation.x},
            {"y", msg->info.origin.orientation.y},
            {"z", msg->info.origin.orientation.z},
            {"w", msg->info.origin.orientation.w}
          }}
      }},
    {"occupancy_ratio", ratio}
  };
}

nlohmann::json ContextGathererNode::compose_geo_payload(
  const geographic_msgs::msg::GeoPointStamped::ConstSharedPtr & msg) const
{
  return {
    {"frame_id", msg->header.frame_id},
    {"stamp", {
        {"sec", msg->header.stamp.sec},
        {"nanosec", msg->header.stamp.nanosec}
      }},
    {"position", {
        {"latitude_deg", msg->position.latitude},
        {"longitude_deg", msg->position.longitude},
        {"altitude_m", msg->position.altitude}
      }}
  };
}

bool ContextGathererNode::is_stale(
  const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now) const
{
  if (data_timeout_sec_ <= 0.0) {
    return false;
  }
  const rclcpp::Time message_time{stamp};
  const auto age = now - message_time;
  return age.seconds() > data_timeout_sec_;
}

std::string ContextGathererNode::encode_image_preview(const sensor_msgs::msg::Image & image) const
{
  try {
    auto cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    cv::Mat working = cv_ptr->image;

    if ((image_target_width_ > 0 && working.cols != image_target_width_) ||
      (image_target_height_ > 0 && working.rows != image_target_height_))
    {
      int target_width = image_target_width_ > 0 ? image_target_width_ : working.cols;
      int target_height = image_target_height_ > 0 ? image_target_height_ : working.rows;

      if (image_target_width_ > 0 && image_target_height_ <= 0) {
        target_height = static_cast<int>(
          std::round(static_cast<double>(working.rows) *
          (static_cast<double>(target_width) / static_cast<double>(working.cols))));
      } else if (image_target_height_ > 0 && image_target_width_ <= 0) {
        target_width = static_cast<int>(
          std::round(static_cast<double>(working.cols) *
          (static_cast<double>(target_height) / static_cast<double>(working.rows))));
      }

      if (target_width > 0 && target_height > 0 &&
        (target_width != working.cols || target_height != working.rows))
      {
        cv::Mat resized;
        cv::resize(working, resized, cv::Size(target_width, target_height));
        working = resized;
      }
    }

    std::vector<uint8_t> buffer;
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, image_jpeg_quality_};
    if (!cv::imencode(".jpg", working, buffer, params)) {
      RCLCPP_WARN(get_logger(), "Failed to encode preview image to JPEG");
      return {};
    }
    return base64_encode(buffer);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_WARN(get_logger(), "cv_bridge exception while encoding image preview: %s", ex.what());
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Unexpected exception encoding image preview: %s", ex.what());
  }
  return {};
}

std::string ContextGathererNode::base64_encode(const std::vector<uint8_t> & buffer)
{
  if (buffer.empty()) {
    return {};
  }

  static constexpr char kAlphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string encoded;
  encoded.reserve(((buffer.size() + 2) / 3) * 4);

  size_t i = 0;
  while (i + 2 < buffer.size()) {
    const uint32_t triple =
      (static_cast<uint32_t>(buffer[i]) << 16) |
      (static_cast<uint32_t>(buffer[i + 1]) << 8) |
      static_cast<uint32_t>(buffer[i + 2]);
    encoded.push_back(kAlphabet[(triple >> 18) & 0x3F]);
    encoded.push_back(kAlphabet[(triple >> 12) & 0x3F]);
    encoded.push_back(kAlphabet[(triple >> 6) & 0x3F]);
    encoded.push_back(kAlphabet[triple & 0x3F]);
    i += 3;
  }

  const auto remaining = buffer.size() - i;
  if (remaining == 1) {
    const uint32_t triple = static_cast<uint32_t>(buffer[i]) << 16;
    encoded.push_back(kAlphabet[(triple >> 18) & 0x3F]);
    encoded.push_back(kAlphabet[(triple >> 12) & 0x3F]);
    encoded.push_back('=');
    encoded.push_back('=');
  } else if (remaining == 2) {
    const uint32_t triple =
      (static_cast<uint32_t>(buffer[i]) << 16) |
      (static_cast<uint32_t>(buffer[i + 1]) << 8);
    encoded.push_back(kAlphabet[(triple >> 18) & 0x3F]);
    encoded.push_back(kAlphabet[(triple >> 12) & 0x3F]);
    encoded.push_back(kAlphabet[(triple >> 6) & 0x3F]);
    encoded.push_back('=');
  }

  return encoded;
}

double ContextGathererNode::compute_occupancy_ratio(const nav_msgs::msg::OccupancyGrid & grid)
{
  if (grid.data.empty()) {
    return 0.0;
  }

  size_t occupied = 0;
  size_t considered = 0;
  for (const auto value : grid.data) {
    if (value < 0) {
      continue;
    }
    ++considered;
    if (value >= 50) {
      ++occupied;
    }
  }

  if (considered == 0) {
    return 0.0;
  }

  return static_cast<double>(occupied) / static_cast<double>(considered);
}

}  // namespace context_gatherer
