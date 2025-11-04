#ifndef BT_ACTIONS__NAVIGATE_ACTION_HPP_
#define BT_ACTIONS__NAVIGATE_ACTION_HPP_

#include <chrono>
#include <future>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace bt_actions
{

class NavigateAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr get_node();
  geometry_msgs::msg::PoseStamped resolve_goal_pose();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  std::shared_future<typename GoalHandleNavigateToPose::SharedPtr> goal_future_;
  typename GoalHandleNavigateToPose::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandleNavigateToPose::WrappedResult> result_future_;
  rclcpp::Time start_time_;
  std::string action_name_{"navigate_to_pose"};
  NavigateToPose::Goal current_goal_;
};

}  // namespace bt_actions

#endif  // BT_ACTIONS__NAVIGATE_ACTION_HPP_
