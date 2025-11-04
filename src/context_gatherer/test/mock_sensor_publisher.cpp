#include "mock_sensor_publisher.hpp"

#include <algorithm>
#include <vector>

namespace context_gatherer::test
{
namespace
{
constexpr char kDefaultCameraTopic[] = "/camera/image_raw";
constexpr char kDefaultOdometryTopic[] = "/odometry/global";
constexpr char kDefaultMapTopic[] = "/map";
constexpr char kDefaultGeoTopic[] = "/gps/geo";
}  // namespace

MockSensorPublisherNode::MockSensorPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mock_sensor_publisher", options)
{
  declare_topics();
  create_publishers();

  const auto publish_period =
    std::chrono::milliseconds(declare_parameter<int>("publish_period_ms", 200));

  timer_ = create_wall_timer(publish_period, [this]() { publish_sample(); });
}

void MockSensorPublisherNode::declare_topics()
{
  camera_topic_ = declare_parameter<std::string>("camera_topic", kDefaultCameraTopic);
  odometry_topic_ = declare_parameter<std::string>("odometry_topic", kDefaultOdometryTopic);
  map_topic_ = declare_parameter<std::string>("map_topic", kDefaultMapTopic);
  geo_topic_ = declare_parameter<std::string>("geo_topic", kDefaultGeoTopic);
}

void MockSensorPublisherNode::create_publishers()
{
  image_publisher_ = image_transport::create_publisher(this, camera_topic_);
  odometry_publisher_ =
    create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, rclcpp::QoS(10).best_effort());
  map_publisher_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, rclcpp::QoS(1).transient_local());
  geo_publisher_ =
    create_publisher<geographic_msgs::msg::GeoPointStamped>(geo_topic_, rclcpp::QoS(10).best_effort());
}

void MockSensorPublisherNode::publish_sample()
{
  const auto now = this->now();

  sensor_msgs::msg::Image image_msg;
  image_msg.header.stamp = now;
  image_msg.header.frame_id = "camera_link";
  image_msg.height = 2;
  image_msg.width = 2;
  image_msg.encoding = "rgb8";
  image_msg.step = image_msg.width * 3;
  image_msg.data = {
    255, 0, 0,
    0, 255, 0,
    0, 0, 255,
    255, 255, 0
  };
  image_publisher_.publish(image_msg);

  nav_msgs::msg::Odometry odometry_msg;
  odometry_msg.header.stamp = now;
  odometry_msg.header.frame_id = "odom";
  odometry_msg.pose.pose.position.x = 1.0;
  odometry_msg.pose.pose.position.y = 2.0;
  odometry_msg.pose.pose.position.z = 0.1;
  odometry_msg.pose.pose.orientation.w = 1.0;
  odometry_msg.twist.twist.linear.x = 0.5;
  odometry_msg.twist.twist.angular.z = 0.1;
  odometry_publisher_->publish(odometry_msg);

  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.stamp = now;
  map_msg.header.frame_id = "map";
  map_msg.info.resolution = 0.5;
  map_msg.info.width = 4;
  map_msg.info.height = 4;
  map_msg.info.origin.orientation.w = 1.0;
  map_msg.data.assign(map_msg.info.width * map_msg.info.height, -1);
  for (size_t i = 0; i < map_msg.data.size(); ++i) {
    map_msg.data[i] = (i % 3 == 0) ? 100 : 0;
  }
  map_publisher_->publish(map_msg);

  geographic_msgs::msg::GeoPointStamped geo_msg;
  geo_msg.header.stamp = now;
  geo_msg.header.frame_id = "earth";
  geo_msg.position.latitude = 48.0;
  geo_msg.position.longitude = 11.0;
  geo_msg.position.altitude = 500.0;
  geo_publisher_->publish(geo_msg);
}

}  // namespace context_gatherer::test
