#include "behaviortree_cpp/bt_factory.h"
#include "bt_nodes_llm/blackboard_update_node.hpp"
#include "bt_nodes_llm/thinking_node.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nodes_llm::ThinkingNode>("ThinkingNode");
  factory.registerNodeType<bt_nodes_llm::BlackboardUpdateNode>("BlackboardUpdateNode");
}
