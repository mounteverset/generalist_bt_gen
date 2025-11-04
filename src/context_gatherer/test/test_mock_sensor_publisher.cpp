#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <vector>

#include "mock_sensor_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

namespace context_gatherer::test
{

class MockSensorPublisherFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_rclcpp_context_ = true;
    }
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    node_ = std::make_shared<MockSensorPublisherNode>();
    executor_->add_node(node_);
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    node_.reset();
    executor_.reset();
    if (owns_rclcpp_context_) {
      rclcpp::shutdown();
    }
  }

  bool owns_rclcpp_context_{false};
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<MockSensorPublisherNode> node_;
};

TEST_F(MockSensorPublisherFixture, publishersAreCreated)
{
  const auto topics = node_->get_topic_names_and_types();
  const auto find_topic = [&](const std::string & name) {
      return std::any_of(
        topics.begin(), topics.end(),
        [&](const auto & pair) {return pair.first == name;});
    };

  EXPECT_TRUE(find_topic("/camera/image_raw"));
  EXPECT_TRUE(find_topic("/odometry/global"));
  EXPECT_TRUE(find_topic("/map"));
  EXPECT_TRUE(find_topic("/gps/geo"));
}

TEST_F(MockSensorPublisherFixture, publishesMessagesOverTime)
{
  size_t camera_count = 0;
  size_t odom_count = 0;

  auto dummy_node = std::make_shared<rclcpp::Node>("mock_sensor_listener");
  auto camera_sub = dummy_node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::Image::ConstSharedPtr) { camera_count++; });
  auto odom_sub = dummy_node->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/global", rclcpp::SystemDefaultsQoS(),
    [&](nav_msgs::msg::Odometry::ConstSharedPtr) { odom_count++; });

  executor_->add_node(dummy_node);

  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 500ms &&
    (camera_count == 0 || odom_count == 0))
  {
    executor_->spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_GT(camera_count, 0u);
  EXPECT_GT(odom_count, 0u);

  executor_->remove_node(dummy_node);
}

}  // namespace context_gatherer::test
