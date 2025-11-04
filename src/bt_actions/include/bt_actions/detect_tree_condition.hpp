#ifndef BT_ACTIONS__DETECT_TREE_CONDITION_HPP_
#define BT_ACTIONS__DETECT_TREE_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_actions
{

class DetectTreeCondition : public BT::ConditionNode
{
public:
  DetectTreeCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr get_node();

  rclcpp::Node::SharedPtr node_;
};

}  // namespace bt_actions

#endif  // BT_ACTIONS__DETECT_TREE_CONDITION_HPP_
