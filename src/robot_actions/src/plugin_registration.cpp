#include <behaviortree_cpp/bt_factory.h>

#include "robot_actions/log_temperature.hpp"
#include "robot_actions/move_to.hpp"
#include "robot_actions/restart_node.hpp"
#include "robot_actions/take_picture.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_actions::MoveTo>("MoveTo");
  factory.registerNodeType<robot_actions::TakePicture>("TakePicture");
  factory.registerNodeType<robot_actions::LogTemperature>("LogTemperature");
  factory.registerNodeType<robot_actions::RestartNode>("RestartNode");
}
