#include "robot_actions/get_current_pose.hpp"

#include "robot_actions/common.hpp"

#include <rclcpp/wait_for_message.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace robot_actions
{

GetCurrentPose::GetCurrentPose(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
  node_ = params.nh.lock();
  enable_debug_logging_ = is_debug_logging_enabled(node_);
  if (!node_) {
    return;
  }

  if (node_->has_parameter("get_current_pose_pose_topic")) {
    default_pose_topic_ = node_->get_parameter("get_current_pose_pose_topic").as_string();
  } else {
    default_pose_topic_ = node_->declare_parameter<std::string>(
      "get_current_pose_pose_topic", "");
  }

  if (node_->has_parameter("get_current_pose_odom_topic")) {
    default_odom_topic_ = node_->get_parameter("get_current_pose_odom_topic").as_string();
  } else if (node_->has_parameter("distance_traveled_odom_topic")) {
    default_odom_topic_ = node_->get_parameter("distance_traveled_odom_topic").as_string();
  } else {
    default_odom_topic_ = node_->declare_parameter<std::string>(
      "get_current_pose_odom_topic", "/target/odometry/fused");
  }
}

BT::PortsList GetCurrentPose::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "pose_topic", "",
      "PoseWithCovarianceStamped topic used to read current map pose. Preferred when set."),
    BT::InputPort<int>(
      "pose_timeout_ms", 1000,
      "Maximum time to wait for one pose sample."),
    BT::InputPort<std::string>(
      "odom_topic", "",
      "Odometry topic used when the pose topic has no sample."),
    BT::InputPort<int>(
      "odom_timeout_ms", 1000,
      "Maximum time to wait for one odometry sample."),
    BT::OutputPort<double>("current_x", "Current pose x position."),
    BT::OutputPort<double>("current_y", "Current pose y position."),
    BT::OutputPort<double>("current_yaw", "Current pose yaw."),
    BT::OutputPort<std::string>("current_pose", "Current pose as 'x,y,yaw'."),
    BT::OutputPort<std::string>(
      "current_frame_id", "Frame id from the received pose or odometry."),
    BT::OutputPort<std::string>("sweep_pose_000", "Current x,y and starting yaw."),
    BT::OutputPort<std::string>("sweep_pose_060", "Current x,y with yaw 60 degrees from start."),
    BT::OutputPort<std::string>("sweep_pose_120", "Current x,y with yaw 120 degrees from start."),
    BT::OutputPort<std::string>("sweep_pose_180", "Current x,y with yaw 180 degrees from start."),
    BT::OutputPort<std::string>("sweep_pose_240", "Current x,y with yaw 240 degrees from start."),
    BT::OutputPort<std::string>("sweep_pose_300", "Current x,y with yaw 300 degrees from start."),
    BT::OutputPort<std::string>("sweep_pose_360", "Current x,y with starting yaw after a full turn.")
  };
}

BT::NodeStatus GetCurrentPose::tick()
{
  if (!node_) {
    RCLCPP_ERROR(get_logger(), "GetCurrentPose -> ROS node is unavailable.");
    return BT::NodeStatus::FAILURE;
  }

  auto pose_topic = getInput<std::string>("pose_topic").value_or("");
  if (pose_topic.empty()) {
    pose_topic = default_pose_topic_;
  }
  const auto pose_timeout_ms = std::max(0, getInput<int>("pose_timeout_ms").value_or(1000));

  auto odom_topic = getInput<std::string>("odom_topic").value_or("");
  if (odom_topic.empty()) {
    odom_topic = default_odom_topic_;
  }
  const auto timeout_ms = std::max(0, getInput<int>("odom_timeout_ms").value_or(1000));

  auto wait_node = make_wait_node();

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string frame_id;

  bool have_pose = false;
  if (!pose_topic.empty()) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    const bool received = rclcpp::wait_for_message<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_msg, wait_node, pose_topic, std::chrono::milliseconds(pose_timeout_ms),
      rclcpp::SystemDefaultsQoS());
    if (received) {
      x = pose_msg.pose.pose.position.x;
      y = pose_msg.pose.pose.position.y;
      yaw = yaw_from_pose(pose_msg);
      frame_id = pose_msg.header.frame_id;
      have_pose = true;
    } else {
      RCLCPP_WARN(
        get_logger(), "GetCurrentPose -> no pose on '%s' within %d ms; trying odometry.",
        pose_topic.c_str(), pose_timeout_ms);
    }
  }
  if (!have_pose) {
    nav_msgs::msg::Odometry odom_msg;
    const bool received = rclcpp::wait_for_message<nav_msgs::msg::Odometry>(
      odom_msg, wait_node, odom_topic, std::chrono::milliseconds(timeout_ms),
      rclcpp::SystemDefaultsQoS());
    if (!received) {
      RCLCPP_ERROR(
        get_logger(), "GetCurrentPose -> no odometry received on '%s' within %d ms.",
        odom_topic.c_str(), timeout_ms);
      return BT::NodeStatus::FAILURE;
    }

    x = odom_msg.pose.pose.position.x;
    y = odom_msg.pose.pose.position.y;
    yaw = yaw_from_odom(odom_msg);
    frame_id = odom_msg.header.frame_id;
  }

  if (frame_id.empty()) {
    RCLCPP_ERROR(get_logger(), "GetCurrentPose -> received pose without a frame id.");
    return BT::NodeStatus::FAILURE;
  }

  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    RCLCPP_ERROR(
      get_logger(), "GetCurrentPose -> received non-finite pose values.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("current_x", x);
  setOutput("current_y", y);
  setOutput("current_yaw", yaw);
  setOutput("current_pose", pose_string(x, y, yaw));
  setOutput("current_frame_id", frame_id);
  const double step = std::acos(-1.0) / 3.0;
  setOutput("sweep_pose_000", pose_string(x, y, yaw));
  setOutput("sweep_pose_060", pose_string(x, y, yaw + step));
  setOutput("sweep_pose_120", pose_string(x, y, yaw + 2.0 * step));
  setOutput("sweep_pose_180", pose_string(x, y, yaw + 3.0 * step));
  setOutput("sweep_pose_240", pose_string(x, y, yaw + 4.0 * step));
  setOutput("sweep_pose_300", pose_string(x, y, yaw + 5.0 * step));
  setOutput("sweep_pose_360", pose_string(x, y, yaw + 6.0 * step));

  if (enable_debug_logging_) {
    RCLCPP_INFO(
      get_logger(), "GetCurrentPose -> x=%.3f y=%.3f yaw=%.3f frame='%s'",
      x, y, yaw, frame_id.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

rclcpp::Node::SharedPtr GetCurrentPose::make_wait_node() const
{
  rclcpp::NodeOptions options;
  options.context(node_->get_node_options().context());
  options.use_global_arguments(false);
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  return std::make_shared<rclcpp::Node>(
    "get_current_pose_wait_node", node_->get_namespace(), options);
}

double GetCurrentPose::yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double GetCurrentPose::yaw_from_odom(const nav_msgs::msg::Odometry & odom)
{
  return yaw_from_quaternion(odom.pose.pose.orientation);
}

double GetCurrentPose::yaw_from_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  return yaw_from_quaternion(pose.pose.pose.orientation);
}

std::string GetCurrentPose::pose_string(double x, double y, double yaw)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << x << "," << y << "," << yaw;
  return stream.str();
}

}  // namespace robot_actions
