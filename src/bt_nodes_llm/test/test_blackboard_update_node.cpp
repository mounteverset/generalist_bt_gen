#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "bt_nodes_llm/blackboard_update_node.hpp"
#include "llm_interface/srv/llm_query.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace
{

std::string wrap_bt(const std::string & node_xml)
{
  return std::string(
    R"(<root BTCPP_format=")" "4" R"(" main_tree_to_execute="Main">")") +
         "\n  <BehaviorTree ID=\"Main\">\n    " + node_xml + "\n  </BehaviorTree>\n</root>";
}

}  // namespace

class BlackboardUpdateNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    service_node_ = std::make_shared<rclcpp::Node>("blackboard_update_service");
    executor_.add_node(service_node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    service_node_.reset();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr service_node_;
  std::thread spin_thread_;
};

TEST_F(BlackboardUpdateNodeTest, UpdatesMultipleBlackboardKeys)
{
  using llm_interface::srv::LLMQuery;

  const std::string service_name = "/blackboard_update_test";
  auto service = service_node_->create_service<LLMQuery>(
    service_name,
    [](const std::shared_ptr<LLMQuery::Request> request,
    std::shared_ptr<LLMQuery::Response> response)
    {
      EXPECT_EQ(request->prompt, "Provide new mission parameters");
      response->success = true;
      response->response_json = R"({"waypoints":["A","B"],"retry_limit":3,"active":true})";
      response->reasoning = "Provided updated parameters";
    });
  (void)service;

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt_nodes_llm::BlackboardUpdateNode>("BlackboardUpdateNode");

  const std::string xml =
    wrap_bt(
    "<BlackboardUpdateNode service_name=\"" + service_name +
    "\" timeout_ms=\"2000\" target_namespace=\"llm_test\" raw_response_key=\"stored_raw\"/>"
    "");

  auto tree = factory.createTreeFromText(xml);
  auto blackboard = tree.rootBlackboard();

  auto client_node = std::make_shared<rclcpp::Node>("blackboard_update_client");
  blackboard->set("ros_node", client_node);
  blackboard->set("llm_update_prompt", std::string("Provide new mission parameters"));

  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(10));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  std::string stored_raw;
  EXPECT_TRUE(blackboard->get("stored_raw", stored_raw));

  std::string waypoint_list;
  EXPECT_TRUE(blackboard->get("llm_test/waypoints", waypoint_list));
  EXPECT_NE(waypoint_list.find("A"), std::string::npos);

  int64_t retry_limit{0};
  EXPECT_TRUE(blackboard->get("llm_test/retry_limit", retry_limit));
  EXPECT_EQ(retry_limit, 3);

  bool active{false};
  EXPECT_TRUE(blackboard->get("llm_test/active", active));
  EXPECT_TRUE(active);
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
