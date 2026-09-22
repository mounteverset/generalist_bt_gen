#include "robot_actions/distance_traveled.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

TEST(DistanceTraveled, DoesNotShareSubscriptionWithSpinningExecutor)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("distance_traveled_wait_set_test");
  node->declare_parameter<std::string>(
    "distance_traveled_odom_topic", "/configured_odom");
  auto publisher = node->create_publisher<nav_msgs::msg::Odometry>(
    "/configured_odom", rclcpp::SystemDefaultsQoS());

  BT::BehaviorTreeFactory factory;
  BT::RosNodeParams params;
  params.nh = node;
  factory.registerNodeType<robot_actions::DistanceTraveled>("DistanceTraveled", params);
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <DistanceTraveled interval_m="1.0" odom_timeout_ms="3000" />
      </BehaviorTree>
    </root>
  )");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  auto tick_with_odom = [&](double x) {
      std::thread publish([publisher, x]() {
          for (int i = 0; i < 200 && publisher->get_subscription_count() == 0; ++i) {
            std::this_thread::sleep_for(10ms);
          }
          nav_msgs::msg::Odometry message;
          message.pose.pose.position.x = x;
          publisher->publish(message);
        });
      status = tree.tickOnce();
      publish.join();
    };
  EXPECT_NO_THROW({
    tick_with_odom(0.0);
    tick_with_odom(2.0);
  });
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  tree.haltTree();
  executor.cancel();
  spinner.join();
  executor.remove_node(node);
  rclcpp::shutdown();
}
