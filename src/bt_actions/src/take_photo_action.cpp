#include "bt_actions/take_photo_action.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp/exceptions.h"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_actions
{

namespace
{
constexpr char kBlackboardNodeKey[] = "ros_node";
constexpr char kDefaultCameraTopic[] = "/camera/image_raw";
constexpr char kDefaultTargetKey[] = "last_photo";
}  // namespace

TakePhotoAction::TakePhotoAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  start_time_(0)
{
}

BT::PortsList TakePhotoAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("camera_topic", kDefaultCameraTopic, "Camera topic providing images"),
    BT::InputPort<std::string>("target", kDefaultTargetKey, "Blackboard key to store captured image"),
    BT::InputPort<unsigned int>("timeout_ms", 2000U, "Milliseconds to wait for camera message")
  };
}

BT::NodeStatus TakePhotoAction::onStart()
{
  auto node = get_node();

  const auto topic = getInput<std::string>("camera_topic").value_or(
    std::string{kDefaultCameraTopic});
  target_key_ = getInput<std::string>("target").value_or(std::string{kDefaultTargetKey});
  timeout_ms_ = getInput<unsigned int>("timeout_ms").value_or(2000U);

  subscribe_if_needed(topic);

  received_image_ = false;
  last_image_.reset();
  start_time_ = node->now();

  rclcpp::spin_some(node);

  if (received_image_) {
    config().blackboard->set(target_key_, *last_image_);
    RCLCPP_INFO(
      node->get_logger(), "TakePhotoAction [%s]: image captured immediately on topic '%s'",
      name().c_str(), topic.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(
    node->get_logger(), "TakePhotoAction [%s]: waiting for image on '%s'",
    name().c_str(), topic.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakePhotoAction::onRunning()
{
  auto node = get_node();
  rclcpp::spin_some(node);

  if (received_image_) {
    config().blackboard->set(target_key_, *last_image_);
    RCLCPP_INFO(
      node->get_logger(), "TakePhotoAction [%s]: image captured and stored at key '%s'",
      name().c_str(), target_key_.c_str());
    received_image_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_ms_ > 0U) {
    const rclcpp::Duration timeout{std::chrono::milliseconds(timeout_ms_)};
    if (node->now() - start_time_ > timeout) {
      RCLCPP_WARN(
        node->get_logger(),
        "TakePhotoAction [%s]: timed out after %u ms waiting for camera message",
        name().c_str(), timeout_ms_);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void TakePhotoAction::onHalted()
{
  received_image_ = false;
  last_image_.reset();
  target_key_.clear();
}

rclcpp::Node::SharedPtr TakePhotoAction::get_node()
{
  if (!node_) {
    try {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>(kBlackboardNodeKey);
    } catch (const BT::RuntimeError &) {
      throw std::runtime_error(
              "TakePhotoAction requires a rclcpp::Node shared pointer stored in blackboard key '" +
              std::string{kBlackboardNodeKey} + "'");
    }
  }
  return node_;
}

void TakePhotoAction::subscribe_if_needed(const std::string & topic)
{
  auto node = get_node();
  if (subscription_ && topic == current_topic_) {
    return;
  }

  subscription_.reset();
  current_topic_ = topic;
  subscription_ = node->create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&TakePhotoAction::image_callback, this, std::placeholders::_1));
}

void TakePhotoAction::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  last_image_ = msg;
  received_image_ = true;
}

}  // namespace bt_actions
