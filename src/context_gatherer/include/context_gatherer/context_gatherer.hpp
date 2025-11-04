#ifndef CONTEXT_GATHERER__CONTEXT_GATHERER_HPP_
#define CONTEXT_GATHERER__CONTEXT_GATHERER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace context_gatherer
{

class ContextGathererNode : public rclcpp::Node
{
public:
  explicit ContextGathererNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct TimedImage
  {
    sensor_msgs::msg::Image::ConstSharedPtr message;
  };

  struct TimedOdometry
  {
    nav_msgs::msg::Odometry::ConstSharedPtr message;
  };

  struct TimedMap
  {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr message;
  };

  struct TimedGeoPoint
  {
    geographic_msgs::msg::GeoPointStamped::ConstSharedPtr message;
  };

  void declare_and_load_parameters();
  void setup_interfaces();

  void handle_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void handle_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void handle_map(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void handle_geopoint(const geographic_msgs::msg::GeoPointStamped::ConstSharedPtr & msg);
  void handle_get_context(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  nlohmann::json compose_context_payload() const;
  nlohmann::json compose_camera_payload(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;
  nlohmann::json compose_odometry_payload(
    const nav_msgs::msg::Odometry::ConstSharedPtr & msg) const;
  nlohmann::json compose_map_payload(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg) const;
  nlohmann::json compose_geo_payload(
    const geographic_msgs::msg::GeoPointStamped::ConstSharedPtr & msg) const;

  bool is_stale(const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now) const;
  std::string encode_image_preview(const sensor_msgs::msg::Image & image) const;
  static std::string base64_encode(const std::vector<uint8_t> & buffer);
  static double compute_occupancy_ratio(const nav_msgs::msg::OccupancyGrid & grid);

  mutable std::mutex data_mutex_;
  TimedImage latest_image_;
  TimedOdometry latest_odometry_;
  TimedMap latest_map_;
  TimedGeoPoint latest_geo_point_;

  double data_timeout_sec_{5.0};
  std::string camera_topic_;
  std::string odometry_topic_;
  std::string map_topic_;
  std::string geo_topic_;
  std::string transport_hint_;
  int image_jpeg_quality_{70};
  int image_target_width_{320};
  int image_target_height_{0};

  image_transport::Subscriber image_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr geo_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

}  // namespace context_gatherer

#endif  // CONTEXT_GATHERER__CONTEXT_GATHERER_HPP_
