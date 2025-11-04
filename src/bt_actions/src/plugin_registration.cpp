#include "behaviortree_cpp/bt_factory.h"
#include "bt_actions/detect_tree_condition.hpp"
#include "bt_actions/explore_field_action.hpp"
#include "bt_actions/navigate_action.hpp"
#include "bt_actions/take_photo_action.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_actions::NavigateAction>("NavigateAction");
  factory.registerNodeType<bt_actions::TakePhotoAction>("TakePhotoAction");
  factory.registerNodeType<bt_actions::DetectTreeCondition>("DetectTreeCondition");
  factory.registerNodeType<bt_actions::ExploreFieldAction>("ExploreFieldAction");
}
