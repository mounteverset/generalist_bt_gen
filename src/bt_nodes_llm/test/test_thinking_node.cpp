#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "llm_interface/srv/llm_query.hpp"
#include "bt_nodes_llm/thinking_node.hpp"
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

class ThinkingNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    service_node_ = std::make_shared<rclcpp::Node>("thinking_node_test_service");
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

TEST_F(ThinkingNodeTest, StoresDecisionFromLLMResponse)
{
  using llm_interface::srv::LLMQuery;

  const std::string service_name = "/thinking_node_test_service";
  auto service = service_node_->create_service<LLMQuery>(
    service_name,
    [](const std::shared_ptr<LLMQuery::Request> request,
    std::shared_ptr<LLMQuery::Response> response)
    {
      EXPECT_EQ(request->prompt, "Explore the field");
      response->success = true;
      response->response_json = R"({"decision":"ExploreField"})";
      response->reasoning = "Mock reasoning";
    });
  (void)service;

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt_nodes_llm::ThinkingNode>("ThinkingNode");

  const std::string xml =
    wrap_bt(
    "<ThinkingNode service_name=\"" + service_name +
    "\" timeout_ms=\"2000\" response_key=\"stored_response\" decision_key=\"stored_decision\"/>");

  auto tree = factory.createTreeFromText(xml);
  auto blackboard = tree.rootBlackboard();

  auto client_node = std::make_shared<rclcpp::Node>("thinking_node_test_client");
  blackboard->set("ros_node", client_node);
  blackboard->set("user_command", std::string("Explore the field"));
  blackboard->set("execution_context", std::string("{\"weather\":\"sunny\"}"));
  blackboard->set("llm_metadata", std::string("{\"attempt\":1}"));

  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(10));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  std::string decision;
  EXPECT_TRUE(blackboard->get("stored_decision", decision));
  EXPECT_EQ(decision, "ExploreField");

  std::string response;
  EXPECT_TRUE(blackboard->get("stored_response", response));
  EXPECT_FALSE(response.empty());
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
