#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>

namespace robot_actions
{

class TakePicture : public BT::SyncActionNode
{
public:
  TakePicture(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::string build_output_path(const std::string & directory, const std::string & prefix) const;
  bool save_image(const sensor_msgs::msg::Image::SharedPtr & image, const std::string & path);

  rclcpp::Node::SharedPtr node_;
  bool enable_debug_logging_{false};
  std::string default_image_topic_{"/camera/image_raw"};
};

}  // namespace robot_actions
