#pragma once

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace robot_actions
{

class TakePicture : public BT::SyncActionNode
{
public:
  TakePicture(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace robot_actions
