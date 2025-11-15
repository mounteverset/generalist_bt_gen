#pragma once

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace robot_actions
{

class MoveTo : public BT::SyncActionNode
{
public:
  MoveTo(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace robot_actions
