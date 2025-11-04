#include "mock_sensor_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<context_gatherer::test::MockSensorPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
