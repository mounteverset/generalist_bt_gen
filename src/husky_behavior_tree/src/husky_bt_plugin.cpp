#include "behaviortree_cpp/bt_factory.h"
#include "husky_behavior_tree/register_nodes.hpp"

BT_REGISTER_NODES(factory)
{
  husky_behavior_tree::register_behavior_tree_nodes(factory);
}

