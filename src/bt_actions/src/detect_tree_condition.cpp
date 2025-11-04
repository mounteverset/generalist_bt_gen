#include "bt_actions/detect_tree_condition.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp/exceptions.h"
#include "rclcpp/rclcpp.hpp"

namespace bt_actions
{

namespace
{
constexpr char kBlackboardNodeKey[] = "ros_node";
constexpr char kDefaultDetectionKey[] = "tree_detection";
constexpr char kDefaultConfidenceSuffix[] = "_confidence";
constexpr char kAlternateConfidenceSuffix[] = "_confidences";
}  // namespace

DetectTreeCondition::DetectTreeCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList DetectTreeCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>("detection_key", kDefaultDetectionKey, "Blackboard key storing detections"),
    BT::InputPort<double>("confidence_threshold", 0.5, "Minimum confidence threshold for detection"),
    BT::InputPort<unsigned int>("minimum_detections", 1U, "Minimum detections required for success")
  };
}

BT::NodeStatus DetectTreeCondition::tick()
{
  auto node = get_node();
  const auto detection_key =
    getInput<std::string>("detection_key").value_or(std::string{kDefaultDetectionKey});
  const auto confidence_threshold = getInput<double>("confidence_threshold").value_or(0.5);
  const auto minimum_detections =
    getInput<unsigned int>("minimum_detections").value_or(1U);

  auto bb = config().blackboard;
  if (!bb) {
    RCLCPP_ERROR(
      node->get_logger(), "DetectTreeCondition [%s]: no blackboard available",
      name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto fetch = [&](auto & value, const std::string & key) -> bool {
      try {
        return bb->get(key, value);
      } catch (const BT::RuntimeError &) {
        return false;
      }
    };

  std::vector<double> confidences;
  bool has_confidence =
    fetch(confidences, detection_key + kDefaultConfidenceSuffix) ||
    fetch(confidences, detection_key + kAlternateConfidenceSuffix);

  std::vector<geometry_msgs::msg::Pose> detections;
  const bool has_detections = fetch(detections, detection_key);

  if (has_detections) {
    std::size_t count = detections.size();
    if (has_confidence) {
      count = std::count_if(
        confidences.begin(), confidences.end(),
        [confidence_threshold](double c) {return c >= confidence_threshold;});
    }

    if (count >= minimum_detections) {
      RCLCPP_INFO(
        node->get_logger(),
        "DetectTreeCondition [%s]: detected %zu trees (threshold %.2f, minimum %u)",
        name().c_str(), count, confidence_threshold, minimum_detections);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "DetectTreeCondition [%s]: only %zu detections found (minimum %u)",
      name().c_str(), count, minimum_detections);
    return BT::NodeStatus::FAILURE;
  }

  if (has_confidence) {
    const auto count = std::count_if(
      confidences.begin(), confidences.end(),
      [confidence_threshold](double c) {return c >= confidence_threshold;});
    if (count >= minimum_detections) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  bool detection_flag = false;
  if (fetch(detection_flag, detection_key)) {
    return detection_flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  RCLCPP_WARN(
    node->get_logger(),
    "DetectTreeCondition [%s]: detection data not available on blackboard key '%s'",
    name().c_str(), detection_key.c_str());
  return BT::NodeStatus::FAILURE;
}

rclcpp::Node::SharedPtr DetectTreeCondition::get_node()
{
  if (!node_) {
    try {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>(kBlackboardNodeKey);
    } catch (const BT::RuntimeError &) {
      throw std::runtime_error(
              "DetectTreeCondition requires a rclcpp::Node shared pointer stored in blackboard key '" +
              std::string{kBlackboardNodeKey} + "'");
    }
  }
  return node_;
}

}  // namespace bt_actions
