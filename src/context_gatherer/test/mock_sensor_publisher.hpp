#ifndef CONTEXT_GATHERER__TEST__MOCK_SENSOR_PUBLISHER_HPP_
#define CONTEXT_GATHERER__TEST__MOCK_SENSOR_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace context_gatherer::test
{

class MockSensorPublisherNode : public rclcpp::Node
{
public:
  explicit MockSensorPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_topics();
  void create_publishers();
  void publish_sample();

  std::string camera_topic_;
  std::string odometry_topic_;
  std::string map_topic_;
  std::string geo_topic_;

  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher image_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr geo_publisher_;
};

}  // namespace context_gatherer::test

#endif  // CONTEXT_GATHERER__TEST__MOCK_SENSOR_PUBLISHER_HPP_
