#include "robot_actions/restart_node.hpp"

#include "robot_actions/common.hpp"

namespace robot_actions
{

RestartNode::RestartNode(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosServiceNode<std_srvs::srv::SetBool>(name, config, params)
{
}

BT::PortsList RestartNode::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>("node_name")});
}

bool RestartNode::setRequest(Request::SharedPtr & request)
{
  target_node_ = getInput<std::string>("node_name").value_or("unknown_node");
  request->data = true;
  RCLCPP_WARN(get_logger(), "RestartNode → requesting restart for %s", target_node_.c_str());
  return true;
}

BT::NodeStatus RestartNode::onResponseReceived(const Response::SharedPtr & response)
{
  RCLCPP_INFO(
    get_logger(), "RestartNode → response for %s: success=%d message=%s",
    target_node_.c_str(), response->success, response->message.c_str());
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RestartNode::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(get_logger(), "RestartNode → service failure: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace robot_actions
