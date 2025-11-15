#pragma once

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace robot_actions
{

class LogTemperature : public BT::SyncActionNode
{
public:
  LogTemperature(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace robot_actions
