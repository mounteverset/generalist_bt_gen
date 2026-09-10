#include "robot_actions/mavros_mode.hpp"

#include "robot_actions/common.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_actions
{

SetMavrosMode::SetMavrosMode(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosServiceNode<mavros_msgs::srv::SetMode>(name, config, params)
{
  enable_debug_logging_ = is_debug_logging_enabled(node_.lock());
}

BT::PortsList SetMavrosMode::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>(
      "custom_mode", "GUIDED", "ArduPilot custom mode to request")});
}

bool SetMavrosMode::setRequest(Request::SharedPtr & request)
{
  requested_mode_ = getInput<std::string>("custom_mode").value_or("GUIDED");
  if (requested_mode_.empty()) {
    return false;
  }
  request->base_mode = 0;
  request->custom_mode = requested_mode_;
  return true;
}

BT::NodeStatus SetMavrosMode::onResponseReceived(const Response::SharedPtr & response)
{
  if (response->mode_sent) {
    if (enable_debug_logging_) {
      RCLCPP_INFO(get_logger(), "SetMavrosMode -> requested mode %s", requested_mode_.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(get_logger(), "SetMavrosMode -> mode request rejected: %s", requested_mode_.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SetMavrosMode::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(get_logger(), "SetMavrosMode -> service failure: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

SetMavrosArm::SetMavrosArm(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosServiceNode<mavros_msgs::srv::CommandBool>(name, config, params)
{}

BT::PortsList SetMavrosArm::providedPorts()
{
  return providedBasicPorts({BT::InputPort<bool>(
      "arm", true, "True to arm the vehicle, false to disarm it")});
}

bool SetMavrosArm::setRequest(Request::SharedPtr & request)
{
  requested_arm_ = getInput<bool>("arm").value_or(true);
  request->value = requested_arm_;
  return true;
}

BT::NodeStatus SetMavrosArm::onResponseReceived(const Response::SharedPtr & response)
{
  if (response->success) {
    RCLCPP_INFO(
      get_logger(), "SetMavrosArm -> vehicle %s request accepted",
      requested_arm_ ? "arm" : "disarm");
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(
    get_logger(), "SetMavrosArm -> vehicle %s request rejected (result=%u)",
    requested_arm_ ? "arm" : "disarm", static_cast<unsigned>(response->result));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SetMavrosArm::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(get_logger(), "SetMavrosArm -> service failure: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

WaitForMavrosMode::WaitForMavrosMode(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config), node_(params.nh.lock())
{
  if (!node_) {
    throw std::runtime_error("WaitForMavrosMode requires a ROS node");
  }
}

BT::PortsList WaitForMavrosMode::providedPorts()
{
  return {
    BT::InputPort<std::string>("desired_mode", "GUIDED", "Mode reported by MAVROS"),
    BT::InputPort<bool>("require_armed", false, "Also require MAVROS to report armed"),
    BT::InputPort<double>("timeout_sec", 5.0, "Maximum time to wait for the mode"),
    BT::InputPort<std::string>("state_topic", "/mavros/state", "MAVROS state topic")
  };
}

BT::NodeStatus WaitForMavrosMode::onStart()
{
  desired_mode_ = getInput<std::string>("desired_mode").value_or("GUIDED");
  require_armed_ = getInput<bool>("require_armed").value_or(false);
  const auto timeout = getInput<double>("timeout_sec");
  const auto state_topic = getInput<std::string>("state_topic").value_or("/mavros/state");
  if (
    desired_mode_.empty() || state_topic.empty() || !timeout || !std::isfinite(*timeout) ||
    *timeout <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "WaitForMavrosMode -> invalid input");
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    current_mode_.clear();
    current_armed_ = false;
  }
  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(*timeout));
  state_subscription_ = node_->create_subscription<mavros_msgs::msg::State>(
    state_topic,
    rclcpp::QoS(10),
    [this](mavros_msgs::msg::State::ConstSharedPtr message) {
      if (!message) {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        current_mode_ = message->mode;
        current_armed_ = message->armed;
      }
      emitWakeUpSignal();
    });
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForMavrosMode::onRunning()
{
  bool state_matches = false;
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    state_matches = current_mode_ == desired_mode_ && (!require_armed_ || current_armed_);
  }
  if (state_matches) {
    state_subscription_.reset();
    RCLCPP_INFO(
      node_->get_logger(), "WaitForMavrosMode -> vehicle is in %s%s",
      desired_mode_.c_str(), require_armed_ ? " and armed" : "");
    return BT::NodeStatus::SUCCESS;
  }
  if (std::chrono::steady_clock::now() >= deadline_) {
    state_subscription_.reset();
    RCLCPP_ERROR(
      node_->get_logger(), "WaitForMavrosMode -> timed out waiting for %s%s",
      desired_mode_.c_str(), require_armed_ ? " and armed" : "");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForMavrosMode::onHalted()
{
  state_subscription_.reset();
}

WaitForMavrosGpsFix::WaitForMavrosGpsFix(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config), node_(params.nh.lock())
{
  if (!node_) {
    throw std::runtime_error("WaitForMavrosGpsFix requires a ROS node");
  }
}

BT::PortsList WaitForMavrosGpsFix::providedPorts()
{
  return {
    BT::InputPort<double>("timeout_sec", 10.0, "Maximum time to wait for a valid GPS fix"),
    BT::InputPort<std::string>(
      "fix_topic", "/mavros/global_position/raw/fix", "MAVROS raw GPS fix topic")
  };
}

BT::NodeStatus WaitForMavrosGpsFix::onStart()
{
  const auto timeout = getInput<double>("timeout_sec");
  const auto fix_topic = getInput<std::string>("fix_topic").value_or(
    "/mavros/global_position/raw/fix");
  if (!timeout || !std::isfinite(*timeout) || *timeout <= 0.0 || fix_topic.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "WaitForMavrosGpsFix -> invalid input");
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(fix_mutex_);
    have_fix_ = false;
  }
  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(*timeout));
  fix_subscription_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    fix_topic,
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr message) {
      if (
        !message || message->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
        !std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
        message->latitude == 0.0 || message->longitude == 0.0)
      {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(fix_mutex_);
        have_fix_ = true;
      }
      emitWakeUpSignal();
    });
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForMavrosGpsFix::onRunning()
{
  bool have_fix = false;
  {
    std::lock_guard<std::mutex> lock(fix_mutex_);
    have_fix = have_fix_;
  }
  if (have_fix) {
    fix_subscription_.reset();
    RCLCPP_INFO(node_->get_logger(), "WaitForMavrosGpsFix -> valid GPS fix received");
    return BT::NodeStatus::SUCCESS;
  }
  if (std::chrono::steady_clock::now() >= deadline_) {
    fix_subscription_.reset();
    RCLCPP_ERROR(node_->get_logger(), "WaitForMavrosGpsFix -> timed out waiting for GPS fix");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForMavrosGpsFix::onHalted()
{
  fix_subscription_.reset();
}

}  // namespace robot_actions
