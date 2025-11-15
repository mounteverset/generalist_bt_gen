#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

namespace robot_actions
{

class RestartNode : public BT::RosServiceNode<std_srvs::srv::SetBool>
{
public:
  RestartNode(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr & request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string target_node_;
};

}  // namespace robot_actions
