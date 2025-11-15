#include <behaviortree_ros2/plugins.hpp>

#include "robot_actions/log_temperature.hpp"
#include "robot_actions/move_to.hpp"
#include "robot_actions/restart_node.hpp"
#include "robot_actions/take_picture.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<robot_actions::MoveTo>("MoveTo", params);
  factory.registerNodeType<robot_actions::TakePicture>("TakePicture", params);
  factory.registerNodeType<robot_actions::LogTemperature>("LogTemperature", params);
  factory.registerNodeType<robot_actions::RestartNode>("RestartNode", params);
}
