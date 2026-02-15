#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <string>

namespace robot_actions
{

class MoveTo : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  MoveTo(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal & goal) override;
  BT::NodeStatus onFeedback(std::shared_ptr<const Feedback> feedback) override;
  BT::NodeStatus onResultReceived(const WrappedResult & result) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  void onHalt() override;
};

}  // namespace robot_actions
