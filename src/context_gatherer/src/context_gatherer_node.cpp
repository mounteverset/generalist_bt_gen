#include <chrono>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "gen_bt_interfaces/action/gather_context.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nlohmann/json.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

class ContextGathererNode : public rclcpp::Node
{
public:
  using GatherContext = gen_bt_interfaces::action::GatherContext;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GatherContext>;
  using RequirementHandler = std::function<void(json&, std::vector<std::string>&)>;

  ContextGathererNode()
  : Node("context_gatherer")
  {
    // Parameters
    action_name_ = this->declare_parameter<std::string>("action_name", "/context_gatherer/gather");
    output_directory_ = this->declare_parameter<std::string>("output_directory", "/tmp/context_gatherer");
    default_timeout_sec_ = this->declare_parameter<double>("default_timeout_sec", 10.0);
    debug_logging_ = this->declare_parameter<bool>("enable_debug_logging", true);
    default_requirements_str_ = this->declare_parameter<std::string>(
      "default_requirements", "ROBOT_POSE,RGB_IMAGE");
    default_requirements_ = parse_requirements(default_requirements_str_);

    // Create output directory
    std::filesystem::create_directories(output_directory_);

    RCLCPP_INFO(
      get_logger(),
      "Starting context_gatherer (action=%s, output_dir=%s, default_timeout=%.2fs)",
      action_name_.c_str(), output_directory_.c_str(), default_timeout_sec_);

    // Initialize requirement handlers
    init_requirement_handlers();

    // Subscribe to sensor topics
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_odom_ = msg;
      }
    );

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_rgb_ = msg;
      }
    );

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_depth_ = msg;
      }
    );

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10,
      [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_battery_ = msg;
      }
    );

    // Action server
    action_server_ = rclcpp_action::create_server<GatherContext>(
      this,
      action_name_,
      std::bind(&ContextGathererNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ContextGathererNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ContextGathererNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Context gatherer ready with %zu requirement handlers", requirement_handlers_.size());
  }

private:
  // Action server
  rclcpp_action::Server<GatherContext>::SharedPtr action_server_;
  
  // Parameters
  std::string action_name_;
  std::string output_directory_;
  double default_timeout_sec_;
  bool debug_logging_;
  std::string default_requirements_str_;
  std::vector<std::string> default_requirements_;

  // Sensor subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  // Cached sensor data
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::Image::SharedPtr latest_rgb_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::BatteryState::SharedPtr latest_battery_;
  std::mutex data_mutex_;

  // Requirement handlers
  std::map<std::string, RequirementHandler> requirement_handlers_;

  void init_requirement_handlers()
  {
    requirement_handlers_["ROBOT_POSE"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_robot_pose(ctx);
    };

    requirement_handlers_["RGB_IMAGE"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_rgb_image(ctx, uris);
    };

    requirement_handlers_["DEPTH_IMAGE"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_depth_image(ctx, uris);
    };

    requirement_handlers_["BATTERY_STATE"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_battery_state(ctx);
    };
  }

  void handle_robot_pose(json& context_json)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (latest_odom_) {
      context_json["ROBOT_POSE"] = {
        {"x", latest_odom_->pose.pose.position.x},
        {"y", latest_odom_->pose.pose.position.y},
        {"z", latest_odom_->pose.pose.position.z},
        {"orientation", {
          {"x", latest_odom_->pose.pose.orientation.x},
          {"y", latest_odom_->pose.pose.orientation.y},
          {"z", latest_odom_->pose.pose.orientation.z},
          {"w", latest_odom_->pose.pose.orientation.w}
        }},
        {"timestamp", latest_odom_->header.stamp.sec}
      };
      if (debug_logging_) {
        RCLCPP_INFO(get_logger(), "Captured ROBOT_POSE: (%.2f, %.2f, %.2f)",
                    latest_odom_->pose.pose.position.x,
                    latest_odom_->pose.pose.position.y,
                    latest_odom_->pose.pose.position.z);
      }
    } else {
      RCLCPP_WARN(get_logger(), "ROBOT_POSE requested but no odometry data available");
    }
  }

  void handle_rgb_image(json& context_json, std::vector<std::string>& uris)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (latest_rgb_) {
      std::string uri = save_image_to_disk(latest_rgb_, "rgb");
      if (!uri.empty()) {
        uris.push_back(uri);
        context_json["RGB_IMAGE"] = {
          {"uri", uri},
          {"width", latest_rgb_->width},
          {"height", latest_rgb_->height},
          {"encoding", latest_rgb_->encoding},
          {"timestamp", latest_rgb_->header.stamp.sec}
        };
        if (debug_logging_) {
          RCLCPP_INFO(get_logger(), "Captured RGB_IMAGE: %dx%d -> %s",
                      latest_rgb_->width, latest_rgb_->height, uri.c_str());
        }
      }
    } else {
      RCLCPP_WARN(get_logger(), "RGB_IMAGE requested but no image data available");
    }
  }

  void handle_depth_image(json& context_json, std::vector<std::string>& uris)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (latest_depth_) {
      std::string uri = save_image_to_disk(latest_depth_, "depth");
      if (!uri.empty()) {
        uris.push_back(uri);
        context_json["DEPTH_IMAGE"] = {
          {"uri", uri},
          {"width", latest_depth_->width},
          {"height", latest_depth_->height},
          {"encoding", latest_depth_->encoding},
          {"timestamp", latest_depth_->header.stamp.sec}
        };
        if (debug_logging_) {
          RCLCPP_INFO(get_logger(), "Captured DEPTH_IMAGE: %dx%d -> %s",
                      latest_depth_->width, latest_depth_->height, uri.c_str());
        }
      }
    } else {
      RCLCPP_WARN(get_logger(), "DEPTH_IMAGE requested but no depth data available");
    }
  }

  void handle_battery_state(json& context_json)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (latest_battery_) {
      context_json["BATTERY_STATE"] = {
        {"percentage", latest_battery_->percentage},
        {"voltage", latest_battery_->voltage},
        {"current", latest_battery_->current},
        {"charge", latest_battery_->charge},
        {"capacity", latest_battery_->capacity},
        {"power_supply_status", latest_battery_->power_supply_status},
        {"timestamp", latest_battery_->header.stamp.sec}
      };
      if (debug_logging_) {
        RCLCPP_INFO(get_logger(), "Captured BATTERY_STATE: %.1f%%", latest_battery_->percentage);
      }
    } else {
      RCLCPP_WARN(get_logger(), "BATTERY_STATE requested but no battery data available");
    }
  }

  std::string save_image_to_disk(const sensor_msgs::msg::Image::SharedPtr& img, const std::string& prefix)
  {
    try {
      auto now = this->now();
      std::string filename = output_directory_ + "/" + prefix + "_" +
                             std::to_string(now.seconds()) + "_" +
                             std::to_string(now.nanoseconds()) + ".png";

      cv_bridge::CvImagePtr cv_ptr;
      // Try to convert to BGR8 for color images, or handle other encodings
      if (img->encoding == "bgr8" || img->encoding == "rgb8") {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
      } else if (img->encoding == "mono8" || img->encoding.find("16") != std::string::npos) {
        cv_ptr = cv_bridge::toCvCopy(img);
      } else {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
      }

      cv::imwrite(filename, cv_ptr->image);
      return "file://" + filename;

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return "";
    } catch (std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to save image: %s", e.what());
      return "";
    }
  }

  std::vector<std::string> parse_requirements(const std::string & csv)
  {
    std::vector<std::string> result;
    std::string current;
    for (char c : csv) {
      if (c == ',' || c == ';') {
        if (!current.empty()) {
          trim_and_push(result, current);
        }
        current.clear();
      } else {
        current.push_back(c);
      }
    }
    if (!current.empty()) {
      trim_and_push(result, current);
    }
    return result;
  }

  void trim_and_push(std::vector<std::string> & vec, std::string & token)
  {
    auto start = token.find_first_not_of(" \t");
    auto end = token.find_last_not_of(" \t");
    if (start == std::string::npos || end == std::string::npos) {
      return;
    }
    std::string trimmed = token.substr(start, end - start + 1);
    if (!trimmed.empty()) {
      vec.push_back(trimmed);
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GatherContext::Goal> goal)
  {
    if (goal->timeout_sec <= 0.0) {
      RCLCPP_WARN(get_logger(), "Goal timeout not set; using default %.2fs", default_timeout_sec_);
    }
    RCLCPP_INFO(
      get_logger(), "Received GatherContext goal (session=%s, subtree=%s, reqs=%zu)",
      goal->session_id.c_str(), goal->subtree_id.c_str(), goal->context_requirements.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(
      get_logger(), "Cancel requested for session=%s",
      goal_handle->get_goal()->session_id.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&ContextGathererNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GatherContext::Result>();
    const double timeout_sec = goal->timeout_sec > 0.0 ? goal->timeout_sec : default_timeout_sec_;
    const auto start = now_seconds();

    std::vector<std::string> requirements = goal->context_requirements.empty()
      ? default_requirements_
      : goal->context_requirements;

    json context_json;
    std::vector<std::string> attachment_uris;

    auto feedback = std::make_shared<GatherContext::Feedback>();
    feedback->stage = "START";
    feedback->detail = "Collecting context requirements.";
    goal_handle->publish_feedback(feedback);

    for (const auto & req : requirements) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Goal canceled during collection.";
        goal_handle->canceled(result);
        return;
      }

      feedback->stage = "COLLECT";
      feedback->detail = req;
      goal_handle->publish_feedback(feedback);

      if (elapsed_seconds(start) > timeout_sec) {
        result->success = false;
        result->message = "Timeout while collecting context.";
        goal_handle->abort(result);
        return;
      }

      // Call appropriate handler
      if (requirement_handlers_.count(req)) {
        requirement_handlers_[req](context_json, attachment_uris);
      } else {
        RCLCPP_WARN(get_logger(), "Unknown requirement: %s", req.c_str());
      }
    }

    feedback->stage = "FINALIZE";
    feedback->detail = "Building snapshot.";
    goal_handle->publish_feedback(feedback);

    result->context_json = context_json.dump();
    result->attachment_uris = attachment_uris;
    result->success = true;
    result->message = "Context gathered successfully";
    goal_handle->succeed(result);

    RCLCPP_INFO(
      get_logger(),
      "Context gathered for session=%s subtree=%s (reqs=%zu, attachments=%zu)",
      goal->session_id.c_str(), goal->subtree_id.c_str(), requirements.size(), attachment_uris.size());
  }

  double now_seconds() const
  {
    return this->now().seconds();
  }

  double elapsed_seconds(double start_sec) const
  {
    return now_seconds() - start_sec;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContextGathererNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
