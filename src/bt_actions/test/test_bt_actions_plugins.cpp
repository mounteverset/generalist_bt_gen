#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "bt_actions/detect_tree_condition.hpp"
#include "bt_actions/explore_field_action.hpp"
#include "bt_actions/navigate_action.hpp"
#include "bt_actions/take_photo_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

constexpr char kBTHeader[] =
  R"(<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">)";

constexpr char kBTFooter[] = R"(  </BehaviorTree>
</root>)";

std::string wrap_bt_xml(const std::string & node_xml)
{
  return std::string(kBTHeader) + "\n    " + node_xml + "\n" + kBTFooter;
}

}  // namespace

class BTActionsFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("bt_actions_test_node");
  }

  void TearDown() override
  {
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(BTActionsFixture, NavigateActionFailsWhenServerUnavailable)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt_actions::NavigateAction>("NavigateAction");

  const std::string xml = wrap_bt_xml(
    R"(<NavigateAction waypoint="field_entry"
        action_name="navigate_to_pose"
        server_timeout_ms="1"
        timeout_ms="1000"/>)");

  auto tree = factory.createTreeFromText(xml);
  auto blackboard = tree.rootBlackboard();
  blackboard->set("ros_node", node_);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 1.0;
  goal.pose.orientation.w = 1.0;
  blackboard->set("field_entry", goal);

  const auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BTActionsFixture, ExploreFieldActionFailsWhenServerUnavailable)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt_actions::ExploreFieldAction>("ExploreFieldAction");

  const std::string xml = wrap_bt_xml(
    R"(<ExploreFieldAction waypoints_key="exploration_waypoints"
        action_name="follow_waypoints"
        server_timeout_ms="1"
        timeout_ms="1000"/>)");

  auto tree = factory.createTreeFromText(xml);
  auto blackboard = tree.rootBlackboard();
  blackboard->set("ros_node", node_);

  std::vector<geometry_msgs::msg::PoseStamped> waypoints(2);
  for (size_t i = 0; i < waypoints.size(); ++i) {
    auto & pose = waypoints[i];
    pose.header.frame_id = "map";
    pose.pose.position.x = static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
  }
  blackboard->set("exploration_waypoints", waypoints);

  const auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BTActionsFixture, TakePhotoActionStoresImageOnBlackboard)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt_actions::TakePhotoAction>("TakePhotoAction");

  const std::string camera_topic = "/test_camera";
  const std::string target_key = "captured_image";
  const std::string xml = wrap_bt_xml(
    "<TakePhotoAction camera_topic=\"" + camera_topic + "\" target=\"" + target_key +
    "\" timeout_ms=\"200\"/>");

  auto tree = factory.createTreeFromText(xml);
  auto blackboard = tree.rootBlackboard();
  blackboard->set("ros_node", node_);

  auto publisher = node_->create_publisher<sensor_msgs::msg::Image>(
    camera_topic, rclcpp::SensorDataQoS());

  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  sensor_msgs::msg::Image image;
  image.header.stamp = node_->now();
  image.header.frame_id = "camera";
  image.height = 1;
  image.width = 1;
  image.encoding = "mono8";
  image.is_bigendian = 0;
  image.step = 1;
  image.data = {0};

  publisher->publish(image);
  rclcpp::spin_some(node_);

  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  sensor_msgs::msg::Image stored;
  EXPECT_TRUE(blackboard->get(target_key, stored));
  EXPECT_EQ(stored.height, image.height);
  EXPECT_EQ(stored.width, image.width);
}

TEST_F(BTActionsFixture, DetectTreeConditionHonorsConfidenceThreshold)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt_actions::DetectTreeCondition>("DetectTreeCondition");

  const std::string xml = wrap_bt_xml(
    R"(<DetectTreeCondition detection_key="tree_detection"
        confidence_threshold="0.7"
        minimum_detections="2"/>)");

  auto tree = factory.createTreeFromText(xml);
  auto blackboard = tree.rootBlackboard();
  blackboard->set("ros_node", node_);

  std::vector<geometry_msgs::msg::Pose> detections(3);
  detections[0].position.x = 1.0;
  detections[1].position.x = 2.0;
  detections[2].position.x = 3.0;
  std::vector<double> confidences = {0.9, 0.8, 0.3};

  blackboard->set("tree_detection", detections);
  blackboard->set("tree_detection_confidences", confidences);

  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  confidences = {0.4, 0.5, 0.6};
  blackboard->set("tree_detection_confidences", confidences);
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
