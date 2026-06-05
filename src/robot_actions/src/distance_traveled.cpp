#include "robot_actions/distance_traveled.hpp"

#include "robot_actions/common.hpp"

#include <rclcpp/wait_for_message.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace robot_actions
{

DistanceTraveled::DistanceTraveled(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config)
{
  node_ = params.nh.lock();
  enable_debug_logging_ = is_debug_logging_enabled(node_);
  if (!node_) {
    return;
  }

  if (node_->has_parameter("distance_traveled_odom_topic")) {
    default_odom_topic_ = node_->get_parameter("distance_traveled_odom_topic").as_string();
  } else {
    default_odom_topic_ = node_->declare_parameter<std::string>(
      "distance_traveled_odom_topic", "/odom");
  }
}

BT::PortsList DistanceTraveled::providedPorts()
{
  return {
    BT::InputPort<double>(
      "interval_m", 5.0,
      "Return SUCCESS whenever this much odometry distance has been traveled."),
    BT::InputPort<std::string>(
      "odom_topic", "",
      "Odometry topic used to measure traveled distance. Defaults to distance_traveled_odom_topic."),
    BT::InputPort<int>(
      "odom_timeout_ms", 50,
      "Maximum time to wait for a new odometry sample on each BT tick."),
    BT::OutputPort<double>(
      "distance_accumulated_m",
      "Distance accumulated toward the current interval.")
  };
}

BT::NodeStatus DistanceTraveled::onStart()
{
  if (!node_) {
    RCLCPP_ERROR(get_logger(), "DistanceTraveled -> ROS node is unavailable.");
    return BT::NodeStatus::FAILURE;
  }
  if (!read_inputs()) {
    return BT::NodeStatus::FAILURE;
  }

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS(), [](nav_msgs::msg::Odometry::SharedPtr) {});

  if (enable_debug_logging_) {
    RCLCPP_INFO(
      get_logger(), "DistanceTraveled -> waiting for %.2f m on odom '%s'",
      interval_m_, odom_topic_.c_str());
  }
  return update_distance();
}

BT::NodeStatus DistanceTraveled::onRunning()
{
  return update_distance();
}

void DistanceTraveled::onHalted()
{
  odom_sub_.reset();
  reset_measurement_state();
}

bool DistanceTraveled::read_inputs()
{
  interval_m_ = getInput<double>("interval_m").value_or(5.0);
  if (interval_m_ <= 0.0 || !std::isfinite(interval_m_)) {
    RCLCPP_ERROR(get_logger(), "DistanceTraveled -> interval_m must be > 0.");
    return false;
  }

  odom_topic_ = getInput<std::string>("odom_topic").value_or("");
  if (odom_topic_.empty()) {
    odom_topic_ = default_odom_topic_;
  }
  odom_timeout_ms_ = std::max(0, getInput<int>("odom_timeout_ms").value_or(50));
  return true;
}

BT::NodeStatus DistanceTraveled::update_distance()
{
  Pose2D pose;
  if (!read_next_odom(pose)) {
    setOutput("distance_accumulated_m", distance_since_success_);
    return BT::NodeStatus::RUNNING;
  }

  if (!last_pose_) {
    last_pose_ = pose;
    setOutput("distance_accumulated_m", distance_since_success_);
    return BT::NodeStatus::RUNNING;
  }

  const double step = distance_between(*last_pose_, pose);
  last_pose_ = pose;
  if (step > 0.0 && std::isfinite(step)) {
    distance_since_success_ += step;
  }
  setOutput("distance_accumulated_m", distance_since_success_);

  if (distance_since_success_ < interval_m_) {
    return BT::NodeStatus::RUNNING;
  }

  if (enable_debug_logging_) {
    RCLCPP_INFO(
      get_logger(), "DistanceTraveled -> interval reached: %.2f / %.2f m",
      distance_since_success_, interval_m_);
  }
  reset_measurement_state();
  last_pose_ = pose;
  setOutput("distance_accumulated_m", 0.0);
  return BT::NodeStatus::SUCCESS;
}

bool DistanceTraveled::read_next_odom(Pose2D & pose)
{
  if (!odom_sub_) {
    return false;
  }

  nav_msgs::msg::Odometry odom_msg;
  const auto timeout = std::chrono::milliseconds(odom_timeout_ms_);
  const bool received = rclcpp::wait_for_message<nav_msgs::msg::Odometry>(
    odom_msg, odom_sub_, node_->get_node_options().context(), timeout);
  if (!received) {
    return false;
  }

  pose.x = odom_msg.pose.pose.position.x;
  pose.y = odom_msg.pose.pose.position.y;
  return true;
}

double DistanceTraveled::distance_between(const Pose2D & a, const Pose2D & b) const
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

void DistanceTraveled::reset_measurement_state()
{
  last_pose_.reset();
  distance_since_success_ = 0.0;
}

}  // namespace robot_actions
