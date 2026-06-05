#include "robot_actions/take_picture.hpp"

#include "robot_actions/common.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace robot_actions
{

TakePicture::TakePicture(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
  node_ = params.nh.lock();
  enable_debug_logging_ = is_debug_logging_enabled(node_);
  if (node_) {
    if (node_->has_parameter("take_photo_image_topic")) {
      default_image_topic_ = node_->get_parameter("take_photo_image_topic").as_string();
    } else {
      default_image_topic_ = node_->declare_parameter<std::string>(
        "take_photo_image_topic", "/camera/image_raw");
    }
  }
}

BT::PortsList TakePicture::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "image_topic", "",
      "RGB image topic to capture from. Defaults to the take_photo_image_topic parameter."),
    BT::InputPort<std::string>(
      "output_directory", "/tmp/robot_actions/photos",
      "Directory where the captured image will be written."),
    BT::InputPort<std::string>(
      "filename_prefix", "photo",
      "Filename prefix used for the saved image."),
    BT::InputPort<int>(
      "timeout_ms", 1000,
      "Maximum time to wait for an RGB image if no frame is available."),
    BT::OutputPort<std::string>("filepath", "Path of the saved image file.")
  };
}

BT::NodeStatus TakePicture::tick()
{
  if (!node_) {
    RCLCPP_ERROR(get_logger(), "TakePicture -> ROS node is unavailable.");
    return BT::NodeStatus::FAILURE;
  }

  auto image_topic = getInput<std::string>("image_topic").value_or("");
  if (image_topic.empty()) {
    image_topic = default_image_topic_;
  }
  auto output_directory = getInput<std::string>("output_directory").value_or(
    "/tmp/robot_actions/photos");
  auto filename_prefix = getInput<std::string>("filename_prefix").value_or("photo");
  const auto timeout_ms = getInput<int>("timeout_ms").value_or(1000);

  sensor_msgs::msg::Image image_msg;
  const auto timeout = std::chrono::milliseconds(std::max(0, timeout_ms));
  const bool received = rclcpp::wait_for_message<sensor_msgs::msg::Image>(
    image_msg, node_, image_topic, timeout, rclcpp::SensorDataQoS());

  if (!received) {
    RCLCPP_ERROR(
      get_logger(), "TakePicture -> no image received on topic '%s' within %d ms.",
      image_topic.c_str(), timeout_ms);
    return BT::NodeStatus::FAILURE;
  }

  auto image = std::make_shared<sensor_msgs::msg::Image>(std::move(image_msg));
  const auto output_path = build_output_path(output_directory, filename_prefix);
  if (!save_image(image, output_path)) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("filepath", output_path);
  if (enable_debug_logging_) {
    RCLCPP_INFO(get_logger(), "TakePicture -> saved RGB image to %s", output_path.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

std::string TakePicture::build_output_path(
  const std::string & directory, const std::string & prefix) const
{
  std::filesystem::create_directories(directory);
  const auto now = std::chrono::system_clock::now();
  const auto now_time = std::chrono::system_clock::to_time_t(now);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()).count() % 1000;

  std::tm time_info{};
#if defined(_WIN32)
  localtime_s(&time_info, &now_time);
#else
  localtime_r(&now_time, &time_info);
#endif

  std::stringstream filename;
  filename << prefix << "_" << std::put_time(&time_info, "%Y%m%d_%H%M%S")
           << "_" << std::setw(3) << std::setfill('0') << millis << ".jpg";
  return (std::filesystem::path(directory) / filename.str()).string();
}

bool TakePicture::save_image(
  const sensor_msgs::msg::Image::SharedPtr & image, const std::string & path)
{
  try {
    cv_bridge::CvImagePtr cv_ptr;
    if (image->encoding == "rgb8") {
      cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
    } else if (image->encoding == "bgr8") {
      cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
    } else {
      cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
    }
    if (!cv::imwrite(path, cv_ptr->image)) {
      RCLCPP_ERROR(get_logger(), "TakePicture -> cv::imwrite failed for %s", path.c_str());
      return false;
    }
  } catch (const cv_bridge::Exception & exc) {
    RCLCPP_ERROR(get_logger(), "TakePicture -> cv_bridge exception: %s", exc.what());
    return false;
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(get_logger(), "TakePicture -> failed to save image: %s", exc.what());
    return false;
  }
  return true;
}

}  // namespace robot_actions
