#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/follow_gps_waypoints.hpp>

#include <optional>
#include <string>

namespace robot_actions
{

class MoveToGPS : public BT::RosActionNode<nav2_msgs::action::FollowGPSWaypoints>
{
public:
  MoveToGPS(
    const std::string & name,
    const BT::NodeConfig & config,
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
  double last_latitude_{0.0};
  double last_longitude_{0.0};
};

}  // namespace robot_actions
