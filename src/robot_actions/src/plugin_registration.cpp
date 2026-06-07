#include <behaviortree_ros2/plugins.hpp>

#include "robot_actions/log_temperature.hpp"
#include "robot_actions/move_to.hpp"
#include "robot_actions/distance_traveled.hpp"
#include "robot_actions/find_object_location.hpp"
#include "robot_actions/get_current_pose.hpp"
#include "robot_actions/parse_waypoints.hpp"
#include "robot_actions/restart_node.hpp"
#include "robot_actions/take_picture.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  auto make_params = [&](const std::string & param_name, const std::string & default_value) {
    auto custom_params = params;
    if (auto node = params.nh.lock()) {
      std::string value;
      if (node->has_parameter(param_name)) {
        value = node->get_parameter(param_name).as_string();
      } else {
        value = node->declare_parameter<std::string>(param_name, default_value);
      }
      custom_params.default_port_value = value;
    }
    return custom_params;
  };

  const auto move_params = make_params("nav2_action_name", "/navigate_to_pose");
  const auto log_service_params = make_params("log_temperature_service_name", "/log_temperature");
  const auto picture_params = make_params("take_photo_image_topic", "/camera/image_raw");
  const auto find_object_params = make_params(
    "find_anything_service_name", "/language_processor/find_object_locations");

  factory.registerNodeType<robot_actions::MoveTo>("MoveTo", move_params);
  factory.registerNodeType<robot_actions::TakePicture>("TakePicture", picture_params);
  factory.registerNodeType<robot_actions::TakePicture>("TakePhoto", picture_params);
  factory.registerNodeType<robot_actions::DistanceTraveled>("DistanceTraveled", params);
  factory.registerNodeType<robot_actions::GetCurrentPose>("GetCurrentPose", params);
  factory.registerNodeType<robot_actions::FindObjectLocation>(
    "FindObjectLocation", find_object_params);
  factory.registerNodeType<robot_actions::FindObjectLocation>(
    "FindAnything", find_object_params);
  factory.registerNodeType<robot_actions::LogTemperature>("LogTemperature", log_service_params);
  factory.registerNodeType<robot_actions::RestartNode>("RestartNode", params);
  factory.registerNodeType<robot_actions::ParseWaypoints>("ParseWaypoints");
}
