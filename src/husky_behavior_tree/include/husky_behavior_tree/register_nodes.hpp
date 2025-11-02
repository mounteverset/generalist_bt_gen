#pragma once

#include "behaviortree_cpp/bt_factory.h"

namespace husky_behavior_tree
{

/**
 * @brief Register all Husky Behavior Tree nodes with the provided factory.
 *
 * Nodes expect a shared pointer to an rclcpp::Node stored on the blackboard
 * under the key "node".
 */
void register_behavior_tree_nodes(BT::BehaviorTreeFactory& factory);

}  // namespace husky_behavior_tree

