#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "gen_bt_interfaces/action/gather_context.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/srv/save_map.hpp"
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
  using SaveMap = nav2_msgs::srv::SaveMap;
  using RequirementHandler = std::function<void(json&, std::vector<std::string>&)>;

  ContextGathererNode()
  : Node("context_gatherer")
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "/context_gatherer/gather");
    output_directory_ = this->declare_parameter<std::string>("output_directory", "/tmp/context_gatherer");
    pose_cov_topic_ = this->declare_parameter<std::string>("pose_cov_topic", "");
    slam_map_topic_ = this->declare_parameter<std::string>("slam_map_topic", "/map");
    annotated_map_service_name_ = this->declare_parameter<std::string>(
      "annotated_map_service_name", "/annotated_map_saver/save_map");
    annotated_map_service_timeout_sec_ = this->declare_parameter<double>(
      "annotated_map_service_timeout_sec", 10.0);
    default_timeout_sec_ = this->declare_parameter<double>("default_timeout_sec", 10.0);
    debug_logging_ = this->declare_parameter<bool>("enable_debug_logging", true);
    default_requirements_str_ = this->declare_parameter<std::string>(
      "default_requirements", "ROBOT_POSE,RGB_IMAGE,ANNOTATED_SLAM_MAP_IMAGE");
    default_requirements_ = parse_requirements(default_requirements_str_);

    std::filesystem::create_directories(output_directory_);

    RCLCPP_INFO(
      get_logger(),
      "Starting context_gatherer (action=%s, output_dir=%s, default_timeout=%.2fs)",
      action_name_.c_str(), output_directory_.c_str(), default_timeout_sec_);

    init_requirement_handlers();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_odom_ = msg;
      });
    if (!pose_cov_topic_.empty()) {
      pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_cov_topic_, 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_pose_cov_ = msg;
        });
    }

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_rgb_ = msg;
      });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_depth_ = msg;
      });

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10,
      [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_battery_ = msg;
      });

    slam_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      slam_map_topic_, 1,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_slam_map_ = msg;
      });

    action_server_ = rclcpp_action::create_server<GatherContext>(
      this,
      action_name_,
      std::bind(&ContextGathererNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ContextGathererNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ContextGathererNode::handle_accepted, this, std::placeholders::_1));

    annotated_map_client_ = create_client<SaveMap>(annotated_map_service_name_);

    RCLCPP_INFO(
      get_logger(),
      "Context gatherer ready with %zu requirement handlers and annotated map client %s",
      requirement_handlers_.size(),
      annotated_map_service_name_.c_str());
  }

private:
  struct PoseSnapshot
  {
    bool available{false};
    bool in_map_frame{false};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    std::string frame_id;
    std::string source;
  };

  struct AnnotatedMapArtifacts
  {
    std::string image_path;
    std::string image_uri;
    std::string metadata_path;
    std::string metadata_uri;
  };

  rclcpp_action::Server<GatherContext>::SharedPtr action_server_;
  rclcpp::Client<SaveMap>::SharedPtr annotated_map_client_;

  std::string action_name_;
  std::string output_directory_;
  std::string pose_cov_topic_;
  std::string slam_map_topic_;
  std::string annotated_map_service_name_;
  double annotated_map_service_timeout_sec_;
  double default_timeout_sec_;
  bool debug_logging_;
  std::string default_requirements_str_;
  std::vector<std::string> default_requirements_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slam_map_sub_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_cov_;
  sensor_msgs::msg::Image::SharedPtr latest_rgb_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::BatteryState::SharedPtr latest_battery_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_slam_map_;
  std::mutex data_mutex_;

  std::map<std::string, RequirementHandler> requirement_handlers_;

  void init_requirement_handlers()
  {
    requirement_handlers_["ROBOT_POSE"] = [this](json& ctx, std::vector<std::string>& /*uris*/) {
      handle_robot_pose(ctx);
    };

    requirement_handlers_["RGB_IMAGE"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_rgb_image(ctx, uris);
    };

    requirement_handlers_["DEPTH_IMAGE"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_depth_image(ctx, uris);
    };

    requirement_handlers_["BATTERY_STATE"] = [this](json& ctx, std::vector<std::string>& /*uris*/) {
      handle_battery_state(ctx);
    };

    requirement_handlers_["SLAM_MAP"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_slam_map(ctx, uris);
    };

    requirement_handlers_["SLAM_MAP_IMAGE"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_slam_map_image(ctx, uris);
    };

    auto annotated_handler = [this](json& ctx, std::vector<std::string>& uris) {
      handle_annotated_slam_map_image(ctx, uris);
    };
    requirement_handlers_["ANNOTATED_SLAM_MAP_IMAGE"] = annotated_handler;
    requirement_handlers_["SLAM_MAP_ANNOTATED_IMAGE"] = annotated_handler;
  }

  void append_geo_hint_context(const std::string& geo_hint, json& context_json)
  {
    if (geo_hint.empty()) {
      return;
    }

    try {
      const auto parsed = json::parse(geo_hint);
      if (parsed.is_object()) {
        context_json["REQUEST_HINTS"] = parsed;
      } else {
        context_json["REQUEST_HINTS"] = {
          {"raw", geo_hint},
          {"parsed", parsed}
        };
      }
      return;
    } catch (const std::exception& exc) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to parse geo_hint as JSON, preserving raw value: %s",
        exc.what());
    }

    context_json["REQUEST_HINTS"] = {
      {"raw", geo_hint}
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
        RCLCPP_INFO(
          get_logger(), "Captured ROBOT_POSE: (%.2f, %.2f, %.2f)",
          latest_odom_->pose.pose.position.x,
          latest_odom_->pose.pose.position.y,
          latest_odom_->pose.pose.position.z);
      }
      return;
    }

    if (latest_pose_cov_) {
      context_json["ROBOT_POSE"] = {
        {"x", latest_pose_cov_->pose.pose.position.x},
        {"y", latest_pose_cov_->pose.pose.position.y},
        {"z", latest_pose_cov_->pose.pose.position.z},
        {"orientation", {
          {"x", latest_pose_cov_->pose.pose.orientation.x},
          {"y", latest_pose_cov_->pose.pose.orientation.y},
          {"z", latest_pose_cov_->pose.pose.orientation.z},
          {"w", latest_pose_cov_->pose.pose.orientation.w}
        }},
        {"timestamp", latest_pose_cov_->header.stamp.sec}
      };
      if (debug_logging_) {
        RCLCPP_INFO(
          get_logger(), "Captured ROBOT_POSE (PoseWithCovariance): (%.2f, %.2f, %.2f)",
          latest_pose_cov_->pose.pose.position.x,
          latest_pose_cov_->pose.pose.position.y,
          latest_pose_cov_->pose.pose.position.z);
      }
      return;
    }

    RCLCPP_WARN(get_logger(), "ROBOT_POSE requested but no pose data available");
  }

  void handle_rgb_image(json& context_json, std::vector<std::string>& uris)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_rgb_) {
      RCLCPP_WARN(get_logger(), "RGB_IMAGE requested but no image data available");
      return;
    }

    std::string uri = save_image_to_disk(latest_rgb_, "rgb");
    if (uri.empty()) {
      return;
    }

    uris.push_back(uri);
    context_json["RGB_IMAGE"] = {
      {"uri", uri},
      {"width", latest_rgb_->width},
      {"height", latest_rgb_->height},
      {"encoding", latest_rgb_->encoding},
      {"timestamp", latest_rgb_->header.stamp.sec}
    };
    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured RGB_IMAGE: %dx%d -> %s",
        latest_rgb_->width, latest_rgb_->height, uri.c_str());
    }
  }

  void handle_depth_image(json& context_json, std::vector<std::string>& uris)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_depth_) {
      RCLCPP_WARN(get_logger(), "DEPTH_IMAGE requested but no depth data available");
      return;
    }

    std::string uri = save_image_to_disk(latest_depth_, "depth");
    if (uri.empty()) {
      return;
    }

    uris.push_back(uri);
    context_json["DEPTH_IMAGE"] = {
      {"uri", uri},
      {"width", latest_depth_->width},
      {"height", latest_depth_->height},
      {"encoding", latest_depth_->encoding},
      {"timestamp", latest_depth_->header.stamp.sec}
    };
    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured DEPTH_IMAGE: %dx%d -> %s",
        latest_depth_->width, latest_depth_->height, uri.c_str());
    }
  }

  void handle_battery_state(json& context_json)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_battery_) {
      RCLCPP_WARN(get_logger(), "BATTERY_STATE requested but no battery data available");
      return;
    }

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
  }

  void handle_slam_map(json& context_json, std::vector<std::string>& uris)
  {
    auto map = get_latest_slam_map();
    if (!map) {
      RCLCPP_WARN(get_logger(), "SLAM_MAP requested but no map data available");
      return;
    }

    std::string uri = save_map_to_disk(map);
    if (uri.empty()) {
      return;
    }

    uris.push_back(uri);
    context_json["SLAM_MAP"] = {
      {"uri", uri},
      {"width", map->info.width},
      {"height", map->info.height},
      {"resolution", map->info.resolution},
      {"frame_id", map->header.frame_id},
      {"timestamp", map->header.stamp.sec},
      {"map_metadata", build_map_metadata_json(map, false)}
    };
    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured SLAM_MAP: %ux%u -> %s",
        map->info.width, map->info.height, uri.c_str());
    }
  }

  void handle_slam_map_image(json& context_json, std::vector<std::string>& uris)
  {
    auto map = get_latest_slam_map();
    if (!map) {
      RCLCPP_WARN(get_logger(), "SLAM_MAP_IMAGE requested but no map data available");
      return;
    }

    std::string uri = save_map_image_to_disk(map);
    if (uri.empty()) {
      return;
    }

    uris.push_back(uri);
    context_json["SLAM_MAP_IMAGE"] = {
      {"uri", uri},
      {"width", map->info.width},
      {"height", map->info.height},
      {"resolution", map->info.resolution},
      {"frame_id", map->header.frame_id},
      {"timestamp", map->header.stamp.sec},
      {"map_metadata", build_map_metadata_json(map, false)}
    };
    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured SLAM_MAP_IMAGE: %ux%u -> %s",
        map->info.width, map->info.height, uri.c_str());
    }
  }

  void handle_annotated_slam_map_image(json& context_json, std::vector<std::string>& uris)
  {
    auto map = get_latest_slam_map();
    if (!map) {
      RCLCPP_WARN(get_logger(), "ANNOTATED_SLAM_MAP_IMAGE requested but no map data available");
      return;
    }

    AnnotatedMapArtifacts artifacts = build_annotated_map_artifacts();
    if (!call_annotated_map_service(artifacts)) {
      return;
    }

    uris.push_back(artifacts.image_uri);
    context_json["ANNOTATED_SLAM_MAP_IMAGE"] = {
      {"uri", artifacts.image_uri},
      {"metadata_uri", artifacts.metadata_uri},
      {"width", map->info.width},
      {"height", map->info.height},
      {"resolution", map->info.resolution},
      {"frame_id", map->header.frame_id},
      {"timestamp", map->header.stamp.sec},
      {"map_metadata", build_map_metadata_json(map, true)}
    };
    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured ANNOTATED_SLAM_MAP_IMAGE via service: %s",
        artifacts.image_uri.c_str());
    }
  }

  std::string save_image_to_disk(const sensor_msgs::msg::Image::SharedPtr& img, const std::string& prefix)
  {
    try {
      const std::string filename = build_timestamped_path(prefix, "png");

      cv_bridge::CvImagePtr cv_ptr;
      if (img->encoding == "bgr8" || img->encoding == "rgb8") {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
      } else if (img->encoding == "mono8" || img->encoding.find("16") != std::string::npos) {
        cv_ptr = cv_bridge::toCvCopy(img);
      } else {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
      }

      cv::imwrite(filename, cv_ptr->image);
      return to_file_uri(filename);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return "";
    } catch (std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to save image: %s", e.what());
      return "";
    }
  }

  std::string save_map_to_disk(const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
  {
    try {
      const std::string filename = build_timestamped_path("slam_map", "json");

      json map_json = {
        {"width", map->info.width},
        {"height", map->info.height},
        {"resolution", map->info.resolution},
        {"frame_id", map->header.frame_id},
        {"timestamp", map->header.stamp.sec},
        {"origin", {
          {"position", {
            {"x", map->info.origin.position.x},
            {"y", map->info.origin.position.y},
            {"z", map->info.origin.position.z}
          }},
          {"orientation", {
            {"x", map->info.origin.orientation.x},
            {"y", map->info.origin.orientation.y},
            {"z", map->info.origin.orientation.z},
            {"w", map->info.origin.orientation.w}
          }}
        }},
        {"data", map->data}
      };

      std::ofstream out(filename);
      if (!out) {
        RCLCPP_ERROR(get_logger(), "Failed to open map file for writing: %s", filename.c_str());
        return "";
      }
      out << map_json.dump();
      return to_file_uri(filename);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to save map: %s", e.what());
      return "";
    }
  }

  std::string save_map_image_to_disk(const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
  {
    try {
      const std::string filename = build_timestamped_path("slam_map", "png");
      const int width = static_cast<int>(map->info.width);
      const int height = static_cast<int>(map->info.height);
      if (width <= 0 || height <= 0) {
        RCLCPP_WARN(get_logger(), "SLAM map has invalid dimensions (%d x %d)", width, height);
        return "";
      }

      cv::Mat image(height, width, CV_8UC1);
      const auto& data = map->data;
      const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
      if (data.size() < expected) {
        RCLCPP_WARN(
          get_logger(), "SLAM map data size mismatch (got %zu, expected %zu)",
          data.size(), expected);
      }

      for (int grid_y = 0; grid_y < height; ++grid_y) {
        const int image_y = height - 1 - grid_y;
        for (int x = 0; x < width; ++x) {
          const size_t idx = static_cast<size_t>(grid_y) * static_cast<size_t>(width) + x;
          const int8_t value = idx < data.size() ? data[idx] : -1;
          uint8_t pixel = 127;
          if (value >= 0) {
            const int clamped = std::max(0, std::min(100, static_cast<int>(value)));
            pixel = static_cast<uint8_t>(255 - (clamped * 255) / 100);
          }
          image.at<uint8_t>(image_y, x) = pixel;
        }
      }

      cv::imwrite(filename, image);
      return to_file_uri(filename);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to save map image: %s", e.what());
      return "";
    }
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr get_latest_slam_map()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_slam_map_;
  }

  static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& orientation)
  {
    const double siny_cosp =
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cosy_cosp =
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  PoseSnapshot make_pose_snapshot_from_odom(
    const nav_msgs::msg::Odometry::SharedPtr& odom,
    const std::string& source,
    const std::string& map_frame_id) const
  {
    PoseSnapshot snapshot;
    if (!odom) {
      return snapshot;
    }

    snapshot.available = true;
    snapshot.x = odom->pose.pose.position.x;
    snapshot.y = odom->pose.pose.position.y;
    snapshot.z = odom->pose.pose.position.z;
    snapshot.yaw = quaternion_to_yaw(odom->pose.pose.orientation);
    snapshot.frame_id = odom->header.frame_id;
    snapshot.source = source;
    snapshot.in_map_frame =
      map_frame_id.empty() || snapshot.frame_id.empty() || snapshot.frame_id == map_frame_id;
    return snapshot;
  }

  PoseSnapshot make_pose_snapshot_from_pose_cov(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& pose_cov,
    const std::string& source,
    const std::string& map_frame_id) const
  {
    PoseSnapshot snapshot;
    if (!pose_cov) {
      return snapshot;
    }

    snapshot.available = true;
    snapshot.x = pose_cov->pose.pose.position.x;
    snapshot.y = pose_cov->pose.pose.position.y;
    snapshot.z = pose_cov->pose.pose.position.z;
    snapshot.yaw = quaternion_to_yaw(pose_cov->pose.pose.orientation);
    snapshot.frame_id = pose_cov->header.frame_id;
    snapshot.source = source;
    snapshot.in_map_frame =
      map_frame_id.empty() || snapshot.frame_id.empty() || snapshot.frame_id == map_frame_id;
    return snapshot;
  }

  PoseSnapshot get_best_pose_snapshot(const std::string& map_frame_id)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const PoseSnapshot pose_cov = make_pose_snapshot_from_pose_cov(
      latest_pose_cov_, "pose_with_covariance", map_frame_id);
    if (pose_cov.available && pose_cov.in_map_frame) {
      return pose_cov;
    }

    const PoseSnapshot odom = make_pose_snapshot_from_odom(latest_odom_, "odometry", map_frame_id);
    if (odom.available && odom.in_map_frame) {
      return odom;
    }

    if (pose_cov.available) {
      return pose_cov;
    }
    return odom;
  }

  static double choose_grid_spacing_m(const double resolution)
  {
    const double target_spacing_m = std::max(resolution, resolution * 80.0);
    const double magnitude = std::pow(10.0, std::floor(std::log10(target_spacing_m)));
    for (double factor : {1.0, 2.0, 5.0, 10.0}) {
      const double candidate = factor * magnitude;
      if (candidate >= target_spacing_m) {
        return candidate;
      }
    }
    return magnitude * 10.0;
  }

  json build_pose_metadata_json(const PoseSnapshot& pose) const
  {
    json pose_json = {
      {"available", pose.available},
      {"in_map_frame", pose.in_map_frame},
      {"source", pose.source},
      {"frame_id", pose.frame_id}
    };

    if (pose.available) {
      pose_json["x"] = pose.x;
      pose_json["y"] = pose.y;
      pose_json["z"] = pose.z;
      pose_json["yaw_rad"] = pose.yaw;
    }
    return pose_json;
  }

  json build_map_metadata_json(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const bool include_grid_spacing)
  {
    const PoseSnapshot pose = get_best_pose_snapshot(map->header.frame_id);
    json metadata = {
      {"frame_id", map->header.frame_id},
      {"timestamp", map->header.stamp.sec},
      {"width", map->info.width},
      {"height", map->info.height},
      {"resolution_m_per_px", map->info.resolution},
      {"origin", {
        {"x", map->info.origin.position.x},
        {"y", map->info.origin.position.y},
        {"z", map->info.origin.position.z},
        {"yaw_rad", quaternion_to_yaw(map->info.origin.orientation)}
      }},
      {"robot_pose", build_pose_metadata_json(pose)}
    };

    if (include_grid_spacing) {
      metadata["grid_spacing_m"] = choose_grid_spacing_m(map->info.resolution);
    }

    return metadata;
  }

  AnnotatedMapArtifacts build_annotated_map_artifacts() const
  {
    AnnotatedMapArtifacts artifacts;
    artifacts.metadata_path = build_timestamped_path("annotated_slam_map", "yaml");
    std::filesystem::path image_path(artifacts.metadata_path);
    image_path.replace_extension(".png");
    artifacts.image_path = image_path.string();
    artifacts.metadata_uri = to_file_uri(artifacts.metadata_path);
    artifacts.image_uri = to_file_uri(artifacts.image_path);
    return artifacts;
  }

  bool call_annotated_map_service(const AnnotatedMapArtifacts& artifacts)
  {
    if (!annotated_map_client_->wait_for_service(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(annotated_map_service_timeout_sec_))))
    {
      RCLCPP_ERROR(
        get_logger(), "Annotated map saver service %s is unavailable",
        annotated_map_service_name_.c_str());
      return false;
    }

    auto request = std::make_shared<SaveMap::Request>();
    request->map_topic = slam_map_topic_;
    request->map_url = artifacts.metadata_path;
    request->image_format = "png";
    request->map_mode = "scale";
    request->free_thresh = 0.25f;
    request->occupied_thresh = 0.65f;

    auto future = annotated_map_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(annotated_map_service_timeout_sec_)) !=
        std::future_status::ready)
    {
      RCLCPP_ERROR(
        get_logger(), "Timed out waiting for annotated map saver service response");
      return false;
    }

    const auto response = future.get();
    if (!response || !response->result) {
      RCLCPP_ERROR(get_logger(), "Annotated map saver service reported failure");
      return false;
    }
    return true;
  }

  static std::string to_file_uri(const std::string& path)
  {
    return "file://" + std::filesystem::absolute(path).string();
  }

  std::string build_timestamped_path(const std::string& prefix, const std::string& extension) const
  {
    const auto stamp_ns = this->now().nanoseconds();
    return output_directory_ + "/" + prefix + "_" + std::to_string(stamp_ns) + "." + extension;
  }

  std::vector<std::string> parse_requirements(const std::string& csv)
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

  void trim_and_push(std::vector<std::string>& vec, std::string& token)
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
    const rclcpp_action::GoalUUID&,
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
    append_geo_hint_context(goal->geo_hint, context_json);

    auto feedback = std::make_shared<GatherContext::Feedback>();
    feedback->stage = "START";
    feedback->detail = "Collecting context requirements.";
    goal_handle->publish_feedback(feedback);

    for (const auto& req : requirements) {
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContextGathererNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
