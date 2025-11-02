#include <filesystem>
#include <fstream>
#include <memory>

#include "bt_executor/bt_failure_handler.hpp"
#include "bt_executor/bt_loader.hpp"
#include "bt_executor/bt_monitor.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "gtest/gtest.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

std::filesystem::path write_temporary_tree()
{
  auto tmp = std::filesystem::temp_directory_path() / "bt_executor_test_tree.xml";
  std::ofstream file(tmp);
  file <<
    R"(
<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <AlwaysSuccess name="AlwaysSucceeds"/>
  </BehaviorTree>
</root>)";
  return tmp;
}

}  // namespace

TEST(BTLoaderTest, ThrowsWhenConfigMissing)
{
  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>("bt_loader_test");
  } catch (const rclcpp::exceptions::RCLError & ex) {
    GTEST_SKIP() << "Skipping test due to RCL initialization failure: " << ex.what();
    return;
  }
  BT::BehaviorTreeFactory factory;
  bt_executor::BTLoader loader(*node, factory);
  EXPECT_THROW(loader.configure_from_yaml("nonexistent.yaml"), std::runtime_error);
}

TEST(BTLoaderTest, LoadsTreeFromXml)
{
  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>("bt_loader_parse");
  } catch (const rclcpp::exceptions::RCLError & ex) {
    GTEST_SKIP() << "Skipping test due to RCL initialization failure: " << ex.what();
    return;
  }
  BT::BehaviorTreeFactory factory;
  bt_executor::BTLoader loader(*node, factory);
  auto tree_file = write_temporary_tree();
  EXPECT_NO_THROW(loader.load_tree(tree_file.string()));
}

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
#include "rclcpp/exceptions.hpp"
