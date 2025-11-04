#ifndef BT_ACTIONS__TAKE_PHOTO_ACTION_HPP_
#define BT_ACTIONS__TAKE_PHOTO_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace bt_actions
{

class TakePhotoAction : public BT::StatefulActionNode
{
public:
  TakePhotoAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr get_node();
  void subscribe_if_needed(const std::string & topic);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  sensor_msgs::msg::Image::SharedPtr last_image_;
  rclcpp::Time start_time_;
  bool received_image_{false};
  std::string current_topic_;
  std::string target_key_;
  unsigned int timeout_ms_{2000U};
};

}  // namespace bt_actions

#endif  // BT_ACTIONS__TAKE_PHOTO_ACTION_HPP_
