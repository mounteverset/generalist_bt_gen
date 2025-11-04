#ifndef BT_ACTIONS__EXPLORE_FIELD_ACTION_HPP_
#define BT_ACTIONS__EXPLORE_FIELD_ACTION_HPP_

#include <future>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace bt_actions
{

class ExploreFieldAction : public BT::StatefulActionNode
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  ExploreFieldAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr get_node();
  std::vector<geometry_msgs::msg::PoseStamped> resolve_waypoints();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
  std::shared_future<typename GoalHandleFollowWaypoints::SharedPtr> goal_future_;
  typename GoalHandleFollowWaypoints::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandleFollowWaypoints::WrappedResult> result_future_;
  rclcpp::Time start_time_;
  std::string action_name_{"follow_waypoints"};
  FollowWaypoints::Goal current_goal_;
};

}  // namespace bt_actions

#endif  // BT_ACTIONS__EXPLORE_FIELD_ACTION_HPP_
