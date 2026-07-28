#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/decorators/loop_node.h>

#include <string>

namespace robot_actions
{

class ParseGpsWaypoints : public BT::SyncActionNode
{
public:
  ParseGpsWaypoints(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace robot_actions
