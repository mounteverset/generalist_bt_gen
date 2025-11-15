#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

namespace robot_actions
{

class LogTemperature : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  LogTemperature(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr & request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string current_log_path_;
};

}  // namespace robot_actions
