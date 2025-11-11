#include <memory>

#include "bt_executor/bt_failure_handler.hpp"
#include "bt_executor/bt_monitor.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "gtest/gtest.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(BTMonitorTest, PublishesStatusMessages)
{
  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>("bt_monitor_test");
  } catch (const rclcpp::exceptions::RCLError & ex) {
    GTEST_SKIP() << "Skipping test due to RCL initialization failure: " << ex.what();
    return;
  }
  bt_executor::BTMonitor monitor(*node);
  BT::BehaviorTreeFactory factory;
  auto tree =
    factory.createTreeFromText(
    R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <AlwaysSuccess name="AlwaysSucceeds"/>
  </BehaviorTree>
</root>)");
  EXPECT_NO_THROW(monitor.publish_status(tree));
}

TEST(BTFailureHandlerTest, BuildsFailureReport)
{
  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>("bt_failure_handler_test");
  } catch (const rclcpp::exceptions::RCLError & ex) {
    GTEST_SKIP() << "Skipping test due to RCL initialization failure: " << ex.what();
    return;
  }
  bt_executor::BTFailureHandler handler(*node);
  BT::BehaviorTreeFactory factory;
  auto tree =
    factory.createTreeFromText(
    R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <AlwaysFailure name="AlwaysFails"/>
  </BehaviorTree>
</root>)");
  tree.tickOnce();
  auto report = handler.build_report(tree, BT::NodeStatus::FAILURE, "Unit test failure");
  EXPECT_EQ(report.failure_message, "Unit test failure");
  EXPECT_EQ(report.status, BT::NodeStatus::FAILURE);
  EXPECT_FALSE(report.context.empty());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
