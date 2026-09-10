#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <stepper_interfaces/action/set_depth.hpp>

#include <optional>
#include <string>

namespace robot_actions
{

class SetDepth : public BT::RosActionNode<stepper_interfaces::action::SetDepth>
{
public:
  SetDepth(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal & goal) override;
  BT::NodeStatus onFeedback(std::shared_ptr<const Feedback> feedback) override;
  BT::NodeStatus onResultReceived(const WrappedResult & result) override;
  BT::NodeStatus onFailure(
    BT::ActionNodeErrorCode error,
    const std::optional<WrappedResult> & result) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  void onHalt() override;

private:
  bool enable_debug_logging_{false};
  int last_target_depth_cm_{0};
};

}  // namespace robot_actions
