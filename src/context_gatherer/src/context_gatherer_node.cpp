#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <fstream>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <curl/curl.h>
#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "btcpp_ros2_interfaces/msg/node_status.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "gen_bt_interfaces/action/gather_context.hpp"
#include "gen_bt_interfaces/srv/annotate_satellite_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "language_feature_msgs/srv/find_object_locations.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nlohmann/json.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

class ContextGathererNode : public rclcpp::Node
{
public:
  using GatherContext = gen_bt_interfaces::action::GatherContext;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GatherContext>;
  using SaveMap = nav2_msgs::srv::SaveMap;
  using AnnotateSatelliteMap = gen_bt_interfaces::srv::AnnotateSatelliteMap;
  using FindObjectLocations = language_feature_msgs::srv::FindObjectLocations;
  using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
  using RequirementHandler = std::function<void(json&, std::vector<std::string>&)>;

  ContextGathererNode()
  : Node("context_gatherer")
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "/context_gatherer/gather");
    output_directory_ = this->declare_parameter<std::string>("output_directory", "/tmp/context_gatherer");
    pose_cov_topic_ = this->declare_parameter<std::string>("pose_cov_topic", "");
    gps_fix_topic_ = this->declare_parameter<std::string>("gps_fix_topic", "/gps/fix");
    slam_map_topic_ = this->declare_parameter<std::string>("slam_map_topic", "/map");
    annotated_map_service_name_ = this->declare_parameter<std::string>(
      "annotated_map_service_name", "/annotated_map_saver/save_map");
    annotated_map_service_timeout_sec_ = this->declare_parameter<double>(
      "annotated_map_service_timeout_sec", 10.0);
    satellite_map_annotator_service_name_ = this->declare_parameter<std::string>(
      "satellite_map_annotator_service_name", "/satellite_map_annotator/annotate");
    satellite_map_annotator_timeout_sec_ = this->declare_parameter<double>(
      "satellite_map_annotator_timeout_sec", 10.0);
    find_anything_service_name_ = this->declare_parameter<std::string>(
      "find_anything_service_name", "/language_processor/find_object_locations");
    find_anything_service_timeout_sec_ = this->declare_parameter<double>(
      "find_anything_service_timeout_sec", 10.0);
    find_anything_default_max_results_ = this->declare_parameter<int>(
      "find_anything_default_max_results", 5);
    bt_executor_action_name_ = this->declare_parameter<std::string>(
      "bt_executor_action_name", "/bt_executor/execute_tree");
    rgb360_sweep_tree_id_ = this->declare_parameter<std::string>(
      "rgb360_sweep_tree_id", "360_rgb_sweep.xml");
    rgb360_sweep_timeout_sec_ = this->declare_parameter<double>(
      "rgb360_sweep_timeout_sec", 45.0);
    rgb360_sweep_camera_topic_ = this->declare_parameter<std::string>(
      "rgb360_sweep_camera_topic", "/a200_0000/sensors/camera_0/color/image");
    rgb360_sweep_pose_topic_ = this->declare_parameter<std::string>(
      "rgb360_sweep_pose_topic", "/a200_0000/pose");
    rgb360_sweep_odom_topic_ = this->declare_parameter<std::string>(
      "rgb360_sweep_odom_topic", "/odom");
    rgb360_sweep_image_timeout_ms_ = this->declare_parameter<int>(
      "rgb360_sweep_image_timeout_ms", 1000);
    rgb360_sweep_pose_timeout_ms_ = this->declare_parameter<int>(
      "rgb360_sweep_pose_timeout_ms", 1000);
    rgb360_sweep_odom_timeout_ms_ = this->declare_parameter<int>(
      "rgb360_sweep_odom_timeout_ms", 1000);
    maptiler_api_key_ = this->declare_parameter<std::string>(
      "maptiler_api_key",
      std::getenv("MAPTILER_API_KEY") ? std::getenv("MAPTILER_API_KEY") : "");
    maptiler_api_mode_ = this->declare_parameter<std::string>("maptiler_api_mode", "tiles");
    maptiler_style_id_ = this->declare_parameter<std::string>("maptiler_style_id", "satellite-v2");
    maptiler_tile_zoom_ = this->declare_parameter<int>("maptiler_tile_zoom", 18);
    maptiler_tile_radius_ = this->declare_parameter<int>("maptiler_tile_radius", 1);
    maptiler_request_timeout_sec_ = this->declare_parameter<double>(
      "maptiler_request_timeout_sec", 10.0);
    dynamic_satellite_zoom_enabled_ = this->declare_parameter<bool>(
      "dynamic_satellite_zoom_enabled", true);
    maptiler_overview_min_zoom_ = this->declare_parameter<int>(
      "maptiler_overview_min_zoom", 13);
    maptiler_overview_max_zoom_ = this->declare_parameter<int>(
      "maptiler_overview_max_zoom", 19);
    maptiler_detail_zoom_ = this->declare_parameter<int>(
      "maptiler_detail_zoom", 18);
    maptiler_max_overview_tile_radius_ = this->declare_parameter<int>(
      "maptiler_max_overview_tile_radius", 2);
    maptiler_max_map_artifacts_ = this->declare_parameter<int>(
      "maptiler_max_map_artifacts", 5);
    maptiler_overview_target_max_m_per_px_ = this->declare_parameter<double>(
      "maptiler_overview_target_max_m_per_px", 1.5);
    overpass_enabled_ = this->declare_parameter<bool>("overpass_enabled", true);
    overpass_endpoint_ = this->declare_parameter<std::string>(
      "overpass_endpoint", "https://overpass-api.de/api/interpreter");
    overpass_radius_m_ = this->declare_parameter<double>("overpass_radius_m", 750.0);
    overpass_timeout_sec_ = this->declare_parameter<double>("overpass_timeout_sec", 10.0);
    overpass_max_linear_features_ = this->declare_parameter<int>(
      "overpass_max_linear_features", 40);
    overpass_max_coordinates_per_feature_ = this->declare_parameter<int>(
      "overpass_max_coordinates_per_feature", 80);
    default_timeout_sec_ = this->declare_parameter<double>("default_timeout_sec", 10.0);
    debug_logging_ = this->declare_parameter<bool>("enable_debug_logging", true);
    default_requirements_str_ = this->declare_parameter<std::string>(
      "default_requirements", "ROBOT_POSE,RGB_IMAGE,ANNOTATED_SLAM_MAP_IMAGE");
    default_requirements_ = parse_requirements(default_requirements_str_);

    if (maptiler_api_key_.empty()) {
      const char* env_key = std::getenv("MAPTILER_API_KEY");
      if (env_key != nullptr) {
        maptiler_api_key_ = env_key;
      }
    }

    std::filesystem::create_directories(output_directory_);

    RCLCPP_INFO(
      get_logger(),
      "Starting context_gatherer (action=%s, output_dir=%s, default_timeout=%.2fs)",
      action_name_.c_str(), output_directory_.c_str(), default_timeout_sec_);

    const auto curl_rc = curl_global_init(CURL_GLOBAL_DEFAULT);
    if (curl_rc == CURLE_OK) {
      curl_ready_ = true;
    } else {
      RCLCPP_ERROR(get_logger(), "libcurl initialization failed: %s", curl_easy_strerror(curl_rc));
    }

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
    if (!gps_fix_topic_.empty()) {
      gps_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_fix_topic_, 10,
        [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_gps_fix_ = msg;
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
    satellite_map_annotator_client_ = create_client<AnnotateSatelliteMap>(
      satellite_map_annotator_service_name_);
    find_anything_client_ = create_client<FindObjectLocations>(find_anything_service_name_);
    bt_executor_client_ = rclcpp_action::create_client<ExecuteTree>(
      this, bt_executor_action_name_);

    RCLCPP_INFO(
      get_logger(),
      "Context gatherer ready with %zu requirement handlers, annotated map client %s, satellite annotator %s, FindAnything client %s, BT executor %s",
      requirement_handlers_.size(),
      annotated_map_service_name_.c_str(),
      satellite_map_annotator_service_name_.c_str(),
      find_anything_service_name_.c_str(),
      bt_executor_action_name_.c_str());
  }

  ~ContextGathererNode() override
  {
    if (curl_ready_) {
      curl_global_cleanup();
    }
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

  struct GpsSnapshot
  {
    bool available{false};
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
    int status{0};
    std::string frame_id;
    int64_t stamp_sec{0};
  };

  struct AnnotatedMapArtifacts
  {
    std::string image_path;
    std::string image_uri;
    std::string metadata_path;
    std::string metadata_uri;
  };

  struct SatelliteMapArtifacts
  {
    std::string image_uri;
    std::string metadata_uri;
    json metadata_json{json::object()};
  };

  struct TileCoordinate
  {
    int x{0};
    int y{0};
    int z{0};
  };

  struct SatelliteFetchResult
  {
    bool success{false};
    std::string image_uri;
    json request_json{json::object()};
    json fetch_metadata{json::object()};
    std::string message;
  };

  struct GeoPoint
  {
    double lat{0.0};
    double lon{0.0};
    std::string label;
    std::string kind;
    std::string reason;
  };

  struct GeoBounds
  {
    double north{-std::numeric_limits<double>::infinity()};
    double south{std::numeric_limits<double>::infinity()};
    double east{-std::numeric_limits<double>::infinity()};
    double west{std::numeric_limits<double>::infinity()};

    bool valid() const
    {
      return std::isfinite(north) && std::isfinite(south) &&
        std::isfinite(east) && std::isfinite(west) &&
        north > south && east > west;
    }

    void include(double lat, double lon)
    {
      if (!std::isfinite(lat) || !std::isfinite(lon)) {
        return;
      }
      north = std::max(north, lat);
      south = std::min(south, lat);
      east = std::max(east, lon);
      west = std::min(west, lon);
    }
  };

  struct TileBounds
  {
    int min_x{0};
    int max_x{0};
    int min_y{0};
    int max_y{0};
  };

  struct PlannedMapArtifact
  {
    std::string role;
    std::string reason;
    json request_json{json::object()};
  };

  rclcpp_action::Server<GatherContext>::SharedPtr action_server_;
  rclcpp::Client<SaveMap>::SharedPtr annotated_map_client_;
  rclcpp::Client<AnnotateSatelliteMap>::SharedPtr satellite_map_annotator_client_;
  rclcpp::Client<FindObjectLocations>::SharedPtr find_anything_client_;
  rclcpp_action::Client<ExecuteTree>::SharedPtr bt_executor_client_;

  std::string action_name_;
  std::string output_directory_;
  std::string pose_cov_topic_;
  std::string gps_fix_topic_;
  std::string slam_map_topic_;
  std::string annotated_map_service_name_;
  double annotated_map_service_timeout_sec_;
  std::string satellite_map_annotator_service_name_;
  double satellite_map_annotator_timeout_sec_;
  std::string find_anything_service_name_;
  double find_anything_service_timeout_sec_;
  int find_anything_default_max_results_;
  std::string bt_executor_action_name_;
  std::string rgb360_sweep_tree_id_;
  double rgb360_sweep_timeout_sec_;
  std::string rgb360_sweep_camera_topic_;
  std::string rgb360_sweep_pose_topic_;
  std::string rgb360_sweep_odom_topic_;
  int rgb360_sweep_image_timeout_ms_;
  int rgb360_sweep_pose_timeout_ms_;
  int rgb360_sweep_odom_timeout_ms_;
  std::string maptiler_api_key_;
  std::string maptiler_api_mode_;
  std::string maptiler_style_id_;
  int maptiler_tile_zoom_;
  int maptiler_tile_radius_;
  double maptiler_request_timeout_sec_;
  bool dynamic_satellite_zoom_enabled_;
  int maptiler_overview_min_zoom_;
  int maptiler_overview_max_zoom_;
  int maptiler_detail_zoom_;
  int maptiler_max_overview_tile_radius_;
  int maptiler_max_map_artifacts_;
  double maptiler_overview_target_max_m_per_px_;
  bool overpass_enabled_;
  std::string overpass_endpoint_;
  double overpass_radius_m_;
  double overpass_timeout_sec_;
  int overpass_max_linear_features_;
  int overpass_max_coordinates_per_feature_;
  double default_timeout_sec_;
  bool curl_ready_{false};
  bool debug_logging_;
  std::string default_requirements_str_;
  std::vector<std::string> default_requirements_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slam_map_sub_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_cov_;
  sensor_msgs::msg::NavSatFix::SharedPtr latest_gps_fix_;
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

    requirement_handlers_["GPS_FIX"] = [this](json& ctx, std::vector<std::string>& /*uris*/) {
      handle_gps_fix(ctx);
    };

    requirement_handlers_["ANNOTATED_SLAM_MAP_IMAGE"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_annotated_slam_map_image(ctx, uris);
    };

    auto satellite_handler = [this](json& ctx, std::vector<std::string>& uris) {
      handle_satellite_map(ctx, uris);
    };
    requirement_handlers_["SATELLITE_MAP"] = satellite_handler;
    requirement_handlers_["SATELLITE_TILE"] = satellite_handler;

    requirement_handlers_["OSM_CONTEXT"] = [this](json& ctx, std::vector<std::string>& /*uris*/) {
      handle_osm_context(ctx);
    };

    requirement_handlers_["FIND_ANYTHING"] = [this](json& ctx, std::vector<std::string>& /*uris*/) {
      handle_find_anything(ctx);
    };

    requirement_handlers_["RGB360SWEEP"] = [this](json& ctx, std::vector<std::string>& uris) {
      handle_rgb360_sweep(ctx, uris);
    };
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

  void handle_gps_fix(json& context_json)
  {
    const GpsSnapshot gps = get_latest_gps_snapshot();
    if (!gps.available) {
      RCLCPP_WARN(get_logger(), "GPS_FIX requested but no GPS fix data available");
      return;
    }

    context_json["GPS_FIX"] = {
      {"latitude", gps.latitude},
      {"longitude", gps.longitude},
      {"altitude", gps.altitude},
      {"status", gps.status},
      {"frame_id", gps.frame_id},
      {"timestamp", gps.stamp_sec}
    };
    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured GPS_FIX: (%.6f, %.6f)",
        gps.latitude, gps.longitude);
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

  void handle_satellite_map(json& context_json, std::vector<std::string>& uris)
  {
    json satellite_request = extract_satellite_request(context_json);
    if (!satellite_request.is_object()) {
      satellite_request = json::object();
      if (debug_logging_) {
        RCLCPP_INFO(
          get_logger(),
          "SATELLITE_MAP requested without explicit request hints; deriving fetch from GPS/default parameters");
      }
    }

    if (dynamic_satellite_zoom_enabled_ && should_use_dynamic_satellite_plan(satellite_request)) {
      handle_dynamic_satellite_map(context_json, satellite_request, uris);
      return;
    }

    std::string source_image_uri = extract_first_string(
      satellite_request, {"source_image_uri", "image_uri", "uri"});
    if (source_image_uri.empty()) {
      const std::string source_image_path = extract_first_string(
        satellite_request, {"source_image_path", "image_path"});
      if (!source_image_path.empty()) {
        source_image_uri = to_file_uri(source_image_path);
      }
    }

    json fetch_metadata = json::object();
    if (source_image_uri.empty()) {
      SatelliteFetchResult fetch_result = fetch_satellite_map_from_maptiler(context_json, satellite_request);
      if (!fetch_result.success) {
        RCLCPP_WARN(get_logger(), "SATELLITE_MAP fetch failed: %s", fetch_result.message.c_str());
        return;
      }
      source_image_uri = fetch_result.image_uri;
      satellite_request = fetch_result.request_json;
      fetch_metadata = fetch_result.fetch_metadata;
    }

    if (source_image_uri.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "SATELLITE_MAP requested but no source_image_uri/image_path was provided in geo_hint");
      return;
    }

    SatelliteMapArtifacts artifacts;
    if (!call_satellite_map_annotator(context_json, satellite_request, source_image_uri, artifacts)) {
      return;
    }

    uris.push_back(artifacts.image_uri);
    context_json["SATELLITE_MAP"] = {
      {"uri", artifacts.image_uri},
      {"metadata_uri", artifacts.metadata_uri},
      {"source_image_uri", source_image_uri},
      {"map_metadata", artifacts.metadata_json}
    };

    if (satellite_request.contains("zoom")) {
      context_json["SATELLITE_MAP"]["zoom"] = satellite_request["zoom"];
    }
    if (satellite_request.contains("source")) {
      context_json["SATELLITE_MAP"]["source"] = satellite_request["source"];
    }
    if (!fetch_metadata.empty()) {
      context_json["SATELLITE_MAP"]["fetch_metadata"] = fetch_metadata;
    }

    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured SATELLITE_MAP via service: %s",
        artifacts.image_uri.c_str());
    }
  }

  void handle_osm_context(json& context_json)
  {
    json request = extract_satellite_request(context_json);
    if (!request.is_object()) {
      request = json::object();
    }

    double center_lat = 0.0;
    double center_lon = 0.0;
    if (!resolve_satellite_center(context_json, request, center_lat, center_lon)) {
      RCLCPP_WARN(get_logger(), "OSM_CONTEXT requested but no GPS/geo center is available");
      return;
    }

    const std::string mission_text = extract_mission_text(context_json);
    const double radius_m = std::max(mission_radius_m(mission_text, request), overpass_radius_m_);
    const json overpass = get_or_fetch_overpass_context(context_json, center_lat, center_lon, radius_m);
    if (overpass.empty()) {
      return;
    }

    const json osm_context = build_osm_context_json(overpass, center_lat, center_lon, radius_m);
    save_osm_context_json(osm_context);
    context_json["OSM_CONTEXT"] = osm_context;
    if (debug_logging_) {
      const auto& osm_context = context_json["OSM_CONTEXT"];
      RCLCPP_INFO(
        get_logger(),
        "Captured OSM_CONTEXT: linear=%zu point=%zu area=%zu",
        osm_context.value("linear_features", json::array()).size(),
        osm_context.value("point_features", json::array()).size(),
        osm_context.value("area_features", json::array()).size());
    }
  }

  bool process_satellite_artifact(
    const json& context_json,
    json satellite_request,
    std::vector<std::string>& uris,
    json& artifact_json)
  {
    std::string source_image_uri = extract_first_string(
      satellite_request, {"source_image_uri", "image_uri", "uri"});
    if (source_image_uri.empty()) {
      const std::string source_image_path = extract_first_string(
        satellite_request, {"source_image_path", "image_path"});
      if (!source_image_path.empty()) {
        source_image_uri = to_file_uri(source_image_path);
      }
    }

    json fetch_metadata = json::object();
    if (source_image_uri.empty()) {
      SatelliteFetchResult fetch_result = fetch_satellite_map_from_maptiler(context_json, satellite_request);
      if (!fetch_result.success) {
        RCLCPP_WARN(get_logger(), "SATELLITE_MAP fetch failed: %s", fetch_result.message.c_str());
        return false;
      }
      source_image_uri = fetch_result.image_uri;
      satellite_request = fetch_result.request_json;
      fetch_metadata = fetch_result.fetch_metadata;
    }

    if (source_image_uri.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "SATELLITE_MAP requested but no source_image_uri/image_path was provided in geo_hint");
      return false;
    }

    SatelliteMapArtifacts artifacts;
    if (!call_satellite_map_annotator(context_json, satellite_request, source_image_uri, artifacts)) {
      return false;
    }

    uris.push_back(artifacts.image_uri);
    artifact_json = {
      {"role", satellite_request.value("role", std::string("map"))},
      {"reason", satellite_request.value("reason", std::string())},
      {"uri", artifacts.image_uri},
      {"metadata_uri", artifacts.metadata_uri},
      {"source_image_uri", source_image_uri},
      {"map_metadata", artifacts.metadata_json}
    };
    if (satellite_request.contains("zoom")) {
      artifact_json["zoom"] = satellite_request["zoom"];
    }
    if (satellite_request.contains("tile_radius")) {
      artifact_json["tile_radius"] = satellite_request["tile_radius"];
    }
    if (satellite_request.contains("bounds")) {
      artifact_json["bounds"] = satellite_request["bounds"];
    }
    if (satellite_request.contains("center")) {
      artifact_json["center"] = satellite_request["center"];
    }
    if (!fetch_metadata.empty()) {
      artifact_json["fetch_metadata"] = fetch_metadata;
    }
    return true;
  }

  void handle_dynamic_satellite_map(
    json& context_json,
    const json& satellite_request,
    std::vector<std::string>& uris)
  {
    const std::vector<PlannedMapArtifact> plan = build_dynamic_satellite_plan(
      context_json, satellite_request);
    if (plan.empty()) {
      RCLCPP_WARN(get_logger(), "Dynamic SATELLITE_MAP planning produced no artifacts");
      return;
    }

    json map_artifacts = json::array();
    json primary_artifact;
    for (const auto& planned : plan) {
      json artifact;
      if (!process_satellite_artifact(context_json, planned.request_json, uris, artifact)) {
        continue;
      }
      if (map_artifacts.empty()) {
        primary_artifact = artifact;
      }
      map_artifacts.push_back(artifact);
    }

    if (map_artifacts.empty()) {
      RCLCPP_WARN(get_logger(), "Dynamic SATELLITE_MAP failed to fetch any planned artifacts");
      return;
    }

    context_json["SATELLITE_MAP"] = {
      {"uri", primary_artifact["uri"]},
      {"metadata_uri", primary_artifact["metadata_uri"]},
      {"source_image_uri", primary_artifact["source_image_uri"]},
      {"map_metadata", primary_artifact["map_metadata"]},
      {"map_artifacts", map_artifacts},
      {"tile_plan", {
        {"mode", "dynamic"},
        {"artifact_count", map_artifacts.size()},
        {"overview_min_zoom", maptiler_overview_min_zoom_},
        {"overview_max_zoom", maptiler_overview_max_zoom_},
        {"detail_zoom", maptiler_detail_zoom_},
        {"max_overview_tile_radius", maptiler_max_overview_tile_radius_}
      }}
    };
    if (primary_artifact.contains("zoom")) {
      context_json["SATELLITE_MAP"]["zoom"] = primary_artifact["zoom"];
    }
    if (primary_artifact.contains("fetch_metadata")) {
      context_json["SATELLITE_MAP"]["fetch_metadata"] = primary_artifact["fetch_metadata"];
    }
    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured dynamic SATELLITE_MAP with %zu artifact(s)",
        map_artifacts.size());
    }
  }

  void handle_find_anything(json& context_json)
  {
    json request_json = extract_find_anything_request(context_json);
    const std::vector<std::string> queries = extract_find_anything_queries(context_json, request_json);
    if (queries.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "FIND_ANYTHING requested but no query was provided in request hints or mission text");
      return;
    }

    if (!find_anything_client_->wait_for_service(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(find_anything_service_timeout_sec_))))
    {
      RCLCPP_ERROR(
        get_logger(), "FindAnything service %s is unavailable",
        find_anything_service_name_.c_str());
      return;
    }

    const uint32_t max_results = static_cast<uint32_t>(std::max(
      1,
      std::min(50, extract_int_from_json(
        request_json, "max_results", find_anything_default_max_results_))));

    json query_results = json::array();
    for (const auto& query : queries) {
      auto request = std::make_shared<FindObjectLocations::Request>();
      request->query = query;
      request->max_results = max_results;

      auto future = find_anything_client_->async_send_request(request);
      if (future.wait_for(std::chrono::duration<double>(find_anything_service_timeout_sec_)) !=
          std::future_status::ready)
      {
        RCLCPP_WARN(
          get_logger(), "Timed out waiting for FindAnything response for query '%s'",
          query.c_str());
        query_results.push_back({
          {"query", query},
          {"max_results", max_results},
          {"success", false},
          {"message", "Timed out waiting for service response"},
          {"locations", json::array()}
        });
        continue;
      }

      const auto response = future.get();
      json locations = json::array();
      if (response) {
        for (const auto& location : response->locations) {
          locations.push_back({
            {"frame_id", location.header.frame_id},
            {"timestamp", {
              {"sec", location.header.stamp.sec},
              {"nanosec", location.header.stamp.nanosec}
            }},
            {"point", {
              {"x", location.point.x},
              {"y", location.point.y},
              {"z", location.point.z}
            }}
          });
        }
      }

      query_results.push_back({
        {"query", query},
        {"max_results", max_results},
        {"success", response != nullptr},
        {"result_count", locations.size()},
        {"locations", locations}
      });
    }

    context_json["FIND_ANYTHING"] = {
      {"service_name", find_anything_service_name_},
      {"max_results", max_results},
      {"query_count", query_results.size()},
      {"queries", query_results}
    };
    if (query_results.size() == 1) {
      context_json["FIND_ANYTHING"]["query"] = query_results[0]["query"];
      context_json["FIND_ANYTHING"]["result_count"] = query_results[0].value("result_count", 0);
      context_json["FIND_ANYTHING"]["locations"] = query_results[0]["locations"];
    }

    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured FIND_ANYTHING for %zu query/queries via %s",
        queries.size(), find_anything_service_name_.c_str());
    }
  }

  void handle_rgb360_sweep(json& context_json, std::vector<std::string>& uris)
  {
    const std::string session_id = extract_mission_session_id(context_json);
    const std::string safe_session = sanitize_path_token(
      session_id.empty() ? "unknown" : session_id);
    const std::string sweep_dir = output_directory_ + "/rgb360sweep_" +
      safe_session + "_" + std::to_string(this->now().nanoseconds());
    std::filesystem::create_directories(sweep_dir);

    json payload = {
      {"photo_output_directory", sweep_dir},
      {"camera_topic", rgb360_sweep_camera_topic_},
      {"pose_topic", rgb360_sweep_pose_topic_},
      {"odom_topic", rgb360_sweep_odom_topic_},
      {"image_timeout_ms", rgb360_sweep_image_timeout_ms_},
      {"pose_timeout_ms", rgb360_sweep_pose_timeout_ms_},
      {"odom_timeout_ms", rgb360_sweep_odom_timeout_ms_}
    };

    ExecuteTree::Result::SharedPtr bt_result;
    std::string message;
    const bool success = execute_sweep_tree(payload.dump(), bt_result, message);
    json images = collect_rgb360_sweep_images(sweep_dir, uris);

    context_json["RGB360SWEEP"] = {
      {"success", success},
      {"tree_id", rgb360_sweep_tree_id_},
      {"bt_executor_action", bt_executor_action_name_},
      {"output_directory", sweep_dir},
      {"output_directory_uri", to_file_uri(sweep_dir)},
      {"image_count", images.size()},
      {"images", images},
      {"message", message}
    };

    if (bt_result) {
      context_json["RGB360SWEEP"]["bt_status"] = bt_result->node_status.status;
      context_json["RGB360SWEEP"]["bt_return_message"] = bt_result->return_message;
    }

    if (!success) {
      RCLCPP_WARN(get_logger(), "RGB360SWEEP failed: %s", message.c_str());
      return;
    }

    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Captured RGB360SWEEP with %zu images in %s",
        images.size(), sweep_dir.c_str());
    }
  }

  bool execute_sweep_tree(
    const std::string& payload_json,
    ExecuteTree::Result::SharedPtr& bt_result,
    std::string& message)
  {
    if (!bt_executor_client_) {
      message = "BT executor action client is unavailable";
      return false;
    }

    const auto timeout = std::chrono::duration<double>(rgb360_sweep_timeout_sec_);
    if (!bt_executor_client_->wait_for_action_server(
          std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)))
    {
      message = "BT executor action server unavailable";
      return false;
    }

    ExecuteTree::Goal goal;
    goal.target_tree = rgb360_sweep_tree_id_;
    goal.payload = payload_json;

    auto goal_handle_future = bt_executor_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(timeout) != std::future_status::ready) {
      message = "Timed out sending RGB360SWEEP goal to BT executor";
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      message = "BT executor rejected RGB360SWEEP goal";
      return false;
    }

    auto result_future = bt_executor_client_->async_get_result(goal_handle);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      message = "Timed out waiting for RGB360SWEEP BT result";
      return false;
    }

    const auto wrapped_result = result_future.get();
    bt_result = wrapped_result.result;
    if (!bt_result) {
      message = "BT executor returned no RGB360SWEEP result";
      return false;
    }

    message = bt_result->return_message;
    return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      bt_result->node_status.status == btcpp_ros2_interfaces::msg::NodeStatus::SUCCESS;
  }

  json collect_rgb360_sweep_images(
    const std::string& sweep_dir,
    std::vector<std::string>& uris)
  {
    json images = json::array();
    const std::vector<std::pair<std::string, double>> expected = {
      {"sweep_000", 0.0},
      {"sweep_060", 1.046},
      {"sweep_120", 2.093},
      {"sweep_180", 3.14},
      {"sweep_240", 4.186}
    };

    for (const auto& [prefix, yaw] : expected) {
      const std::string path = find_first_file_with_prefix(sweep_dir, prefix);
      if (path.empty()) {
        images.push_back({
          {"prefix", prefix},
          {"yaw_rad", yaw},
          {"available", false}
        });
        continue;
      }
      const std::string uri = to_file_uri(path);
      uris.push_back(uri);
      images.push_back({
        {"prefix", prefix},
        {"yaw_rad", yaw},
        {"available", true},
        {"path", path},
        {"uri", uri}
      });
    }
    return images;
  }

  std::string find_first_file_with_prefix(
    const std::string& directory,
    const std::string& prefix) const
  {
    if (!std::filesystem::exists(directory)) {
      return "";
    }

    std::vector<std::filesystem::path> matches;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const auto path = entry.path();
      const std::string filename = path.filename().string();
      if (filename.rfind(prefix + "_", 0) == 0 && path.extension() == ".jpg") {
        matches.push_back(path);
      }
    }
    std::sort(matches.begin(), matches.end());
    return matches.empty() ? "" : matches.front().string();
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

  nav_msgs::msg::OccupancyGrid::SharedPtr get_latest_slam_map()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_slam_map_;
  }

  GpsSnapshot get_latest_gps_snapshot()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    GpsSnapshot snapshot;
    if (!latest_gps_fix_) {
      return snapshot;
    }

    snapshot.available = std::isfinite(latest_gps_fix_->latitude) &&
      std::isfinite(latest_gps_fix_->longitude);
    snapshot.latitude = latest_gps_fix_->latitude;
    snapshot.longitude = latest_gps_fix_->longitude;
    snapshot.altitude = latest_gps_fix_->altitude;
    snapshot.status = latest_gps_fix_->status.status;
    snapshot.frame_id = latest_gps_fix_->header.frame_id;
    snapshot.stamp_sec = latest_gps_fix_->header.stamp.sec;
    return snapshot;
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

  bool call_satellite_map_annotator(
    const json& context_json,
    const json& satellite_request,
    const std::string& source_image_uri,
    SatelliteMapArtifacts& artifacts)
  {
    if (!satellite_map_annotator_client_->wait_for_service(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(satellite_map_annotator_timeout_sec_))))
    {
      RCLCPP_ERROR(
        get_logger(), "Satellite map annotator service %s is unavailable",
        satellite_map_annotator_service_name_.c_str());
      return false;
    }

    auto request = std::make_shared<AnnotateSatelliteMap::Request>();
    request->session_id = extract_mission_session_id(context_json);
    request->source_image_uri = source_image_uri;
    request->request_json = satellite_request.dump();

    auto future = satellite_map_annotator_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(satellite_map_annotator_timeout_sec_)) !=
        std::future_status::ready)
    {
      RCLCPP_ERROR(
        get_logger(), "Timed out waiting for satellite map annotator service response");
      return false;
    }

    const auto response = future.get();
    if (!response || !response->success) {
      RCLCPP_ERROR(
        get_logger(), "Satellite map annotator service reported failure: %s",
        response ? response->message.c_str() : "no response");
      return false;
    }

    artifacts.image_uri = response->annotated_image_uri;
    artifacts.metadata_uri = response->metadata_uri;
    if (!response->metadata_json.empty()) {
      try {
        artifacts.metadata_json = json::parse(response->metadata_json);
      } catch (const std::exception& exc) {
        RCLCPP_WARN(
          get_logger(), "Failed to parse satellite annotator metadata_json: %s", exc.what());
      }
    }
    return true;
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

  static std::string extract_first_string(
    const json& object,
    const std::vector<std::string>& keys)
  {
    if (!object.is_object()) {
      return "";
    }
    for (const auto& key : keys) {
      const auto it = object.find(key);
      if (it != object.end() && it->is_string()) {
        return it->get<std::string>();
      }
    }
    return "";
  }

  static bool looks_like_satellite_request(const json& value)
  {
    if (!value.is_object()) {
      return false;
    }
    return value.contains("source_image_uri") || value.contains("image_uri") || value.contains("uri") ||
      value.contains("source_image_path") || value.contains("image_path") ||
      value.contains("bounds") || value.contains("bbox");
  }

  static json find_nested_satellite_request(const json& value)
  {
    if (!value.is_object()) {
      return json();
    }

    for (const auto& key : {"SATELLITE_MAP", "satellite_map", "SATELLITE_TILE", "satellite_tile"}) {
      const auto it = value.find(key);
      if (it != value.end() && it->is_object()) {
        return *it;
      }
    }

    if (looks_like_satellite_request(value)) {
      return value;
    }

    const auto original_context = value.find("original_goal_context");
    if (original_context != value.end()) {
      const json nested = find_nested_satellite_request(*original_context);
      if (!nested.is_null()) {
        return nested;
      }
    }

    return json();
  }

  static json extract_satellite_request(const json& context_json)
  {
    const auto request_hints = context_json.find("REQUEST_HINTS");
    if (request_hints == context_json.end()) {
      return json();
    }
    return find_nested_satellite_request(*request_hints);
  }

  static json find_nested_find_anything_request(const json& value)
  {
    if (!value.is_object()) {
      if (value.is_string()) {
        return {{"query", value.get<std::string>()}};
      }
      return json();
    }

    for (const auto& key : {"FIND_ANYTHING", "find_anything", "OBJECT_QUERY", "object_query"}) {
      const auto it = value.find(key);
      if (it != value.end()) {
        if (it->is_object()) {
          return *it;
        }
        if (it->is_string()) {
          return {{"query", it->get<std::string>()}};
        }
      }
    }

    const auto original_context = value.find("original_goal_context");
    if (original_context != value.end()) {
      const json nested = find_nested_find_anything_request(*original_context);
      if (!nested.is_null()) {
        return nested;
      }
    }

    return json();
  }

  static json extract_find_anything_request(const json& context_json)
  {
    const auto request_hints = context_json.find("REQUEST_HINTS");
    if (request_hints == context_json.end()) {
      return json::object();
    }
    const json request = find_nested_find_anything_request(*request_hints);
    return request.is_object() ? request : json::object();
  }

  static std::string extract_mission_text(const json& context_json)
  {
    const auto request_hints = context_json.find("REQUEST_HINTS");
    if (request_hints == context_json.end() || !request_hints->is_object()) {
      return "";
    }
    const auto mission_request = request_hints->find("MISSION_REQUEST");
    if (mission_request != request_hints->end() && mission_request->is_object()) {
      const auto mission_text = mission_request->find("mission_text");
      if (mission_text != mission_request->end() && mission_text->is_string()) {
        return mission_text->get<std::string>();
      }
    }
    return "";
  }

  static std::vector<std::string> extract_find_anything_queries(
    const json& context_json,
    const json& request_json)
  {
    std::vector<std::string> queries;
    if (request_json.is_object()) {
      const auto queries_it = request_json.find("queries");
      if (queries_it != request_json.end() && queries_it->is_array()) {
        for (const auto& entry : *queries_it) {
          if (entry.is_string()) {
            const std::string query = entry.get<std::string>();
            if (!query.empty()) {
              queries.push_back(query);
            }
          } else if (entry.is_object()) {
            const std::string query = extract_first_string(
              entry, {"query", "text", "object", "object_query"});
            if (!query.empty()) {
              queries.push_back(query);
            }
          }
        }
      }

      const std::string single_query = extract_first_string(
        request_json, {"query", "text", "object", "object_query"});
      if (!single_query.empty()) {
        queries.push_back(single_query);
      }
    }

    if (queries.empty()) {
      const std::string mission_text = extract_mission_text(context_json);
      if (!mission_text.empty()) {
        queries.push_back(mission_text);
      }
    }
    return queries;
  }

  static std::string extract_mission_session_id(const json& context_json)
  {
    const auto request_hints = context_json.find("REQUEST_HINTS");
    if (request_hints == context_json.end() || !request_hints->is_object()) {
      return "unknown";
    }
    const auto mission_request = request_hints->find("MISSION_REQUEST");
    if (mission_request != request_hints->end() && mission_request->is_object()) {
      const auto session_id = mission_request->find("session_id");
      if (session_id != mission_request->end() && session_id->is_string()) {
        return session_id->get<std::string>();
      }
    }
    return "unknown";
  }

  static constexpr double pi()
  {
    return 3.14159265358979323846;
  }

  static size_t curl_write_callback(void* contents, size_t size, size_t nmemb, void* userp)
  {
    const size_t total = size * nmemb;
    auto* buffer = static_cast<std::vector<unsigned char>*>(userp);
    const auto* bytes = static_cast<const unsigned char*>(contents);
    buffer->insert(buffer->end(), bytes, bytes + total);
    return total;
  }

  static std::optional<double> extract_number_from_json(
    const json& object,
    const std::vector<std::string>& keys)
  {
    if (!object.is_object()) {
      return std::nullopt;
    }
    for (const auto& key : keys) {
      const auto it = object.find(key);
      if (it == object.end()) {
        continue;
      }
      if (it->is_number()) {
        return it->get<double>();
      }
      if (it->is_string()) {
        try {
          return std::stod(it->get<std::string>());
        } catch (...) {
          continue;
        }
      }
    }
    return std::nullopt;
  }

  static int extract_int_from_json(
    const json& object,
    const std::string& key,
    int fallback)
  {
    if (!object.is_object()) {
      return fallback;
    }
    const auto it = object.find(key);
    if (it == object.end()) {
      return fallback;
    }
    if (it->is_number_integer()) {
      return it->get<int>();
    }
    if (it->is_number()) {
      return static_cast<int>(std::round(it->get<double>()));
    }
    if (it->is_string()) {
      try {
        return std::stoi(it->get<std::string>());
      } catch (...) {
        return fallback;
      }
    }
    return fallback;
  }

  static double extract_double_from_json(
    const json& object,
    const std::string& key,
    double fallback)
  {
    if (!object.is_object()) {
      return fallback;
    }
    const auto it = object.find(key);
    if (it == object.end()) {
      return fallback;
    }
    if (it->is_number()) {
      return it->get<double>();
    }
    if (it->is_string()) {
      try {
        return std::stod(it->get<std::string>());
      } catch (...) {
        return fallback;
      }
    }
    return fallback;
  }

  static std::string lowercase_copy(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value;
  }

  static bool contains_any(const std::string& text, const std::vector<std::string>& needles)
  {
    for (const auto& needle : needles) {
      if (text.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  static bool should_use_dynamic_satellite_plan(const json& satellite_request)
  {
    if (!satellite_request.is_object()) {
      return true;
    }
    if (!extract_first_string(
          satellite_request, {"source_image_uri", "image_uri", "uri", "source_image_path", "image_path"}).empty())
    {
      return false;
    }
    return !satellite_request.contains("zoom") &&
      !satellite_request.contains("tile_radius") &&
      !satellite_request.contains("tile_bounds");
  }

  static bool extract_bounds_from_json(const json& object, GeoBounds& bounds)
  {
    if (!object.is_object()) {
      return false;
    }
    json bounds_json;
    if (object.contains("bounds") && object["bounds"].is_object()) {
      bounds_json = object["bounds"];
    } else if (object.contains("bbox") && object["bbox"].is_object()) {
      bounds_json = object["bbox"];
    } else {
      return false;
    }

    const auto north = extract_number_from_json(bounds_json, {"north", "max_lat"});
    const auto south = extract_number_from_json(bounds_json, {"south", "min_lat"});
    const auto east = extract_number_from_json(bounds_json, {"east", "max_lon"});
    const auto west = extract_number_from_json(bounds_json, {"west", "min_lon"});
    if (!north.has_value() || !south.has_value() || !east.has_value() || !west.has_value()) {
      return false;
    }
    bounds = {};
    bounds.include(north.value(), east.value());
    bounds.include(south.value(), west.value());
    return bounds.valid();
  }

  static GeoBounds bounds_from_center_radius(double lat, double lon, double radius_m)
  {
    constexpr double earth_radius_m = 6371000.0;
    const double lat_delta = radius_m / earth_radius_m * 180.0 / pi();
    const double lon_delta = radius_m /
      (earth_radius_m * std::max(0.1, std::cos(lat * pi() / 180.0))) * 180.0 / pi();
    GeoBounds bounds;
    bounds.include(lat - lat_delta, lon - lon_delta);
    bounds.include(lat + lat_delta, lon + lon_delta);
    return bounds;
  }

  static GeoBounds pad_bounds_m(const GeoBounds& input, double padding_m)
  {
    if (!input.valid()) {
      return input;
    }
    const double center_lat = (input.north + input.south) / 2.0;
    constexpr double earth_radius_m = 6371000.0;
    const double lat_delta = padding_m / earth_radius_m * 180.0 / pi();
    const double lon_delta = padding_m /
      (earth_radius_m * std::max(0.1, std::cos(center_lat * pi() / 180.0))) * 180.0 / pi();
    GeoBounds bounds;
    bounds.include(input.south - lat_delta, input.west - lon_delta);
    bounds.include(input.north + lat_delta, input.east + lon_delta);
    return bounds;
  }

  static double mission_radius_m(const std::string& mission_text, const json& satellite_request)
  {
    const double explicit_radius = extract_double_from_json(satellite_request, "radius_m", -1.0);
    if (explicit_radius > 0.0) {
      return explicit_radius;
    }
    const std::string text = lowercase_copy(mission_text);
    if (contains_any(text, {"circumvent", "around", "loop", "lake", "perimeter"})) {
      return 850.0;
    }
    if (contains_any(text, {"survey", "explore", "cover", "field", "park", "area"})) {
      return 350.0;
    }
    return 150.0;
  }

  bool download_text_post(
    const std::string& url,
    const std::string& post_body,
    double timeout_sec,
    std::string& response_text,
    std::string& error_message)
  {
    if (!curl_ready_) {
      error_message = "libcurl is not initialized";
      return false;
    }

    std::vector<unsigned char> buffer;
    CURL* curl = curl_easy_init();
    if (!curl) {
      error_message = "curl_easy_init failed";
      return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(post_body.size()));
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &ContextGathererNode::curl_write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "generalist_bt_gen/context_gatherer");
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_sec * 1000.0));
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/x-www-form-urlencoded");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    const CURLcode rc = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (rc != CURLE_OK) {
      error_message = curl_easy_strerror(rc);
      return false;
    }
    if (http_code < 200 || http_code >= 300) {
      error_message = "HTTP " + std::to_string(http_code);
      return false;
    }
    response_text.assign(buffer.begin(), buffer.end());
    return true;
  }

  json fetch_overpass_context(double lat, double lon, double radius_m)
  {
    if (!overpass_enabled_ || overpass_endpoint_.empty()) {
      return json::object();
    }

    std::ostringstream query;
    query << "[out:json][timeout:" << std::max(1, static_cast<int>(std::round(overpass_timeout_sec_))) << "];";
    query << "(";
    query << "way(around:" << radius_m << "," << lat << "," << lon
          << ")[\"highway\"~\"^(path|track|service|footway|cycleway|steps|pedestrian|living_street|residential|unclassified|tertiary)$\"];";
    query << "way(around:" << radius_m << "," << lat << "," << lon << ")[\"barrier\"];";
    query << "node(around:" << radius_m << "," << lat << "," << lon << ")[\"barrier\"];";
    query << "way(around:" << radius_m << "," << lat << "," << lon
          << ")[\"natural\"~\"^(water|wood|scrub)$\"];";
    query << "relation(around:" << radius_m << "," << lat << "," << lon << ")[\"natural\"=\"water\"];";
    query << "way(around:" << radius_m << "," << lat << "," << lon
          << ")[\"landuse\"~\"^(meadow|grass|recreation_ground)$\"];";
    query << "way(around:" << radius_m << "," << lat << "," << lon << ")[\"leisure\"=\"park\"];";
    query << "node(around:" << radius_m << "," << lat << "," << lon << ")[\"amenity\"];";
    query << "node(around:" << radius_m << "," << lat << "," << lon << ")[\"emergency\"];";
    query << ");out geom center;";

    CURL* escape_curl = curl_easy_init();
    if (!escape_curl) {
      return json::object();
    }
    char* escaped = curl_easy_escape(escape_curl, query.str().c_str(), 0);
    std::string post_body = "data=";
    if (escaped) {
      post_body += escaped;
      curl_free(escaped);
    }
    curl_easy_cleanup(escape_curl);

    std::string response_text;
    std::string error_message;
    if (!download_text_post(overpass_endpoint_, post_body, overpass_timeout_sec_, response_text, error_message)) {
      RCLCPP_WARN(get_logger(), "Overpass context request failed: %s", error_message.c_str());
      return json::object();
    }
    try {
      return json::parse(response_text);
    } catch (const std::exception& exc) {
      RCLCPP_WARN(get_logger(), "Failed to parse Overpass response: %s", exc.what());
      return json::object();
    }
  }

  json get_or_fetch_overpass_context(json& context_json, double lat, double lon, double radius_m)
  {
    constexpr const char* cache_key = "__OVERPASS_CACHE";
    const auto cache_it = context_json.find(cache_key);
    if (cache_it != context_json.end() && cache_it->is_object()) {
      double cached_lat = 0.0;
      double cached_lon = 0.0;
      const bool has_center = cache_it->contains("center") &&
        cache_it->at("center").is_object() &&
        extract_lat_lon_from_entry(cache_it->at("center"), cached_lat, cached_lon);
      const double cached_radius = cache_it->value("radius_m", 0.0);
      const auto response_it = cache_it->find("response");
      if (has_center && cached_radius >= radius_m &&
          haversine_distance_m(cached_lat, cached_lon, lat, lon) < 5.0 &&
          response_it != cache_it->end() && response_it->is_object())
      {
        return *response_it;
      }
    }

    json response = fetch_overpass_context(lat, lon, radius_m);
    if (!response.empty()) {
      context_json[cache_key] = {
        {"center", {{"lat", lat}, {"lon", lon}}},
        {"radius_m", radius_m},
        {"response", response}
      };
    }
    return response;
  }

  static std::optional<GeoPoint> element_center_point(const json& element)
  {
    if (!element.is_object()) {
      return std::nullopt;
    }
    if (element.contains("center") && element["center"].is_object()) {
      double lat = 0.0;
      double lon = 0.0;
      if (extract_lat_lon_from_entry(element["center"], lat, lon)) {
        return GeoPoint{lat, lon, "", "", ""};
      }
    }
    double lat = 0.0;
    double lon = 0.0;
    if (extract_lat_lon_from_entry(element, lat, lon)) {
      return GeoPoint{lat, lon, "", "", ""};
    }
    if (element.contains("geometry") && element["geometry"].is_array() && !element["geometry"].empty()) {
      double lat_sum = 0.0;
      double lon_sum = 0.0;
      int count = 0;
      for (const auto& point : element["geometry"]) {
        if (extract_lat_lon_from_entry(point, lat, lon)) {
          lat_sum += lat;
          lon_sum += lon;
          ++count;
        }
      }
      if (count > 0) {
        return GeoPoint{
          lat_sum / static_cast<double>(count),
          lon_sum / static_cast<double>(count),
          "",
          "",
          ""};
      }
    }
    return std::nullopt;
  }

  static void include_element_geometry(const json& element, GeoBounds& bounds)
  {
    if (!element.is_object()) {
      return;
    }
    if (element.contains("geometry") && element["geometry"].is_array()) {
      for (const auto& point : element["geometry"]) {
        double lat = 0.0;
        double lon = 0.0;
        if (extract_lat_lon_from_entry(point, lat, lon)) {
          bounds.include(lat, lon);
        }
      }
      return;
    }
    const auto center = element_center_point(element);
    if (center.has_value()) {
      bounds.include(center->lat, center->lon);
    }
  }

  static std::string classify_overpass_element(const json& element)
  {
    if (!element.is_object() || !element.contains("tags") || !element["tags"].is_object()) {
      return "";
    }
    const auto& tags = element["tags"];
    if (tags.contains("barrier")) {
      return "barrier";
    }
    if (tags.value("highway", std::string()) == "steps") {
      return "steps";
    }
    if (tags.contains("highway")) {
      return "route";
    }
    if (tags.value("natural", std::string()) == "water") {
      return "water";
    }
    if (tags.contains("natural")) {
      return "avoid_natural";
    }
    if (tags.contains("landuse") || tags.contains("leisure")) {
      return "area";
    }
    if (tags.contains("amenity") || tags.contains("emergency")) {
      return "landmark";
    }
    return "";
  }

  static json marker_json_from_point(const GeoPoint& point, int id)
  {
    json marker = {
      {"id", id},
      {"lat", point.lat},
      {"lon", point.lon},
      {"label", std::to_string(id)}
    };
    if (!point.kind.empty()) {
      marker["type"] = point.kind;
    }
    if (!point.reason.empty()) {
      marker["description"] = point.reason;
    }
    return marker;
  }

  static void enrich_marker_from_overpass_element(json& marker, const json& element, const std::string& kind)
  {
    if (!element.is_object()) {
      return;
    }
    if (element.contains("type") && element["type"].is_string()) {
      marker["osm_type"] = element["type"];
    }
    if (element.contains("id")) {
      marker["osm_id"] = element["id"];
    }
    if (!element.contains("tags") || !element["tags"].is_object()) {
      return;
    }
    const auto& tags = element["tags"];
    marker["tags"] = tags;
    if (tags.contains("name") && tags["name"].is_string()) {
      marker["name"] = tags["name"];
    }

    std::string description = "OSM " + kind + " feature";
    if (kind == "barrier" && tags.contains("barrier")) {
      description = "Barrier: " + tags["barrier"].get<std::string>();
      marker["recommended_use"] = "avoid_or_confirm";
    } else if (kind == "steps") {
      description = "Steps; avoid for Husky ground navigation unless explicitly intended.";
      marker["recommended_use"] = "avoid";
    } else if (kind == "water") {
      description = "Water body; avoid for Husky ground navigation.";
      marker["recommended_use"] = "avoid";
    } else if (kind == "landmark") {
      description = "Operator-visible landmark";
      marker["recommended_use"] = "reference";
    }
    if (tags.contains("name") && tags["name"].is_string()) {
      description += " (" + tags["name"].get<std::string>() + ")";
    }
    marker["description"] = description;
  }

  static std::string highway_kind(const std::string& highway)
  {
    if (highway == "steps") {
      return "steps";
    }
    if (highway == "residential" || highway == "unclassified" || highway == "tertiary" ||
        highway == "living_street")
    {
      return "street";
    }
    if (highway == "path" || highway == "track" || highway == "service" ||
        highway == "footway" || highway == "cycleway" || highway == "pedestrian")
    {
      return "path";
    }
    return "route";
  }

  static std::string linear_feature_label(const std::string& kind, int path_id, int street_id)
  {
    if (kind == "street") {
      return "S" + std::to_string(street_id);
    }
    return "P" + std::to_string(path_id);
  }

  static std::string linear_feature_description(const std::string& kind, const json& tags)
  {
    const std::string highway = tags.value("highway", std::string("way"));
    std::string description = "OSM " + highway + " geometry; ";
    if (kind == "steps") {
      description += "avoid for Husky ground navigation unless explicitly intended.";
    } else if (kind == "street") {
      description += "street-like segment, verify access and safety before using for robot navigation.";
    } else {
      description += "candidate route segment, verify traversability before execution.";
    }
    if (tags.contains("name") && tags["name"].is_string()) {
      description += " (" + tags["name"].get<std::string>() + ")";
    }
    return description;
  }

  static json sampled_geometry_coordinates(
    const json& element,
    int max_coordinates,
    bool& truncated,
    int& original_count)
  {
    json coordinates = json::array();
    truncated = false;
    original_count = 0;
    if (!element.contains("geometry") || !element["geometry"].is_array()) {
      return coordinates;
    }

    std::vector<GeoPoint> points;
    for (const auto& point : element["geometry"]) {
      double lat = 0.0;
      double lon = 0.0;
      if (extract_lat_lon_from_entry(point, lat, lon)) {
        points.push_back(GeoPoint{lat, lon, "", "", ""});
      }
    }
    original_count = static_cast<int>(points.size());
    if (points.empty()) {
      return coordinates;
    }

    const int limit = std::max(2, max_coordinates);
    truncated = original_count > limit;
    const int output_count = truncated ? limit : original_count;
    int last_index = -1;
    for (int i = 0; i < output_count; ++i) {
      int index = i;
      if (truncated) {
        index = static_cast<int>(std::round(
          static_cast<double>(i) * static_cast<double>(original_count - 1) /
          static_cast<double>(output_count - 1)));
      }
      if (index == last_index) {
        continue;
      }
      last_index = index;
      coordinates.push_back({{"lat", points[index].lat}, {"lon", points[index].lon}});
    }
    return coordinates;
  }

  static json bounds_json_from_element(const json& element)
  {
    GeoBounds bounds;
    include_element_geometry(element, bounds);
    if (!bounds.valid()) {
      return json::object();
    }
    return {
      {"north", bounds.north},
      {"south", bounds.south},
      {"east", bounds.east},
      {"west", bounds.west}
    };
  }

  static json base_osm_feature_json(
    const json& element,
    const std::string& label,
    const std::string& kind,
    const std::string& description)
  {
    json feature = {
      {"label", label},
      {"kind", kind},
      {"description", description}
    };
    if (element.contains("type") && element["type"].is_string()) {
      feature["osm_type"] = element["type"];
    }
    if (element.contains("id")) {
      feature["osm_id"] = element["id"];
    }
    if (element.contains("tags") && element["tags"].is_object()) {
      feature["tags"] = element["tags"];
      const auto& tags = element["tags"];
      if (tags.contains("name") && tags["name"].is_string()) {
        feature["name"] = tags["name"];
      }
    }
    const auto center = element_center_point(element);
    if (center.has_value()) {
      feature["center"] = {{"lat", center->lat}, {"lon", center->lon}};
    }
    return feature;
  }

  json build_osm_context_json(
    const json& overpass,
    double center_lat,
    double center_lon,
    double radius_m) const
  {
    json linear_features = json::array();
    json point_features = json::array();
    json area_features = json::array();
    int path_id = 1;
    int street_id = 1;
    int point_id = 1;
    int area_id = 1;
    bool truncated = false;
    int skipped_linear = 0;
    int skipped_point = 0;
    int skipped_area = 0;

    const int max_linear = std::max(0, overpass_max_linear_features_);
    const int max_other_features = max_linear;
    const int max_coordinates = std::max(2, overpass_max_coordinates_per_feature_);

    if (overpass.is_object() && overpass.contains("elements") && overpass["elements"].is_array()) {
      for (const auto& element : overpass["elements"]) {
        if (!element.is_object() || !element.contains("tags") || !element["tags"].is_object()) {
          continue;
        }
        const auto& tags = element["tags"];
        const std::string osm_type = element.value("type", std::string());
        const std::string kind = classify_overpass_element(element);
        if (osm_type == "way" && tags.contains("highway") &&
            tags["highway"].is_string() && element.contains("geometry"))
        {
          if (static_cast<int>(linear_features.size()) >= max_linear) {
            truncated = true;
            ++skipped_linear;
            continue;
          }
          const std::string feature_kind = highway_kind(tags["highway"].get<std::string>());
          const std::string label = linear_feature_label(feature_kind, path_id, street_id);
          if (feature_kind == "street") {
            ++street_id;
          } else {
            ++path_id;
          }
          bool coordinates_truncated = false;
          int original_count = 0;
          json coordinates = sampled_geometry_coordinates(
            element, max_coordinates, coordinates_truncated, original_count);
          if (coordinates.empty()) {
            continue;
          }
          json feature = base_osm_feature_json(
            element, label, feature_kind, linear_feature_description(feature_kind, tags));
          feature["coordinates"] = coordinates;
          feature["original_coordinate_count"] = original_count;
          if (coordinates_truncated) {
            feature["coordinates_truncated"] = true;
            truncated = true;
          }
          if (feature_kind == "steps") {
            feature["recommended_use"] = "avoid";
          } else {
            feature["recommended_use"] = "candidate_route_verify";
          }
          linear_features.push_back(feature);
          continue;
        }

        if (kind == "barrier" || kind == "landmark") {
          if (static_cast<int>(point_features.size()) >= max_other_features) {
            truncated = true;
            ++skipped_point;
            continue;
          }
          if (!element_center_point(element).has_value()) {
            continue;
          }
          json feature = base_osm_feature_json(
            element,
            "M" + std::to_string(point_id++),
            kind,
            kind == "barrier" ? "OSM barrier; avoid or confirm before navigation." :
            "OSM landmark; useful for visual/geographic reference.");
          if (kind == "barrier") {
            feature["recommended_use"] = "avoid_or_confirm";
          } else {
            feature["recommended_use"] = "reference";
          }
          point_features.push_back(feature);
          continue;
        }

        if (kind == "water" || kind == "avoid_natural" || kind == "area") {
          if (static_cast<int>(area_features.size()) >= max_other_features) {
            truncated = true;
            ++skipped_area;
            continue;
          }
          json feature = base_osm_feature_json(
            element,
            "A" + std::to_string(area_id++),
            kind,
            kind == "water" ? "OSM water body; avoid for Husky ground navigation." :
            "OSM area feature; use as geographic context, not an executable route.");
          json bounds = bounds_json_from_element(element);
          if (!bounds.empty()) {
            feature["bounds"] = bounds;
          }
          feature["recommended_use"] = kind == "water" ? "avoid" : "context";
          area_features.push_back(feature);
        }
      }
    }

    return {
      {"provider", "overpass"},
      {"center", {{"lat", center_lat}, {"lon", center_lon}}},
      {"radius_m", radius_m},
      {"linear_features", linear_features},
      {"point_features", point_features},
      {"area_features", area_features},
      {"feature_counts", {
        {"linear", linear_features.size()},
        {"point", point_features.size()},
        {"area", area_features.size()},
        {"overpass_elements", overpass.value("elements", json::array()).size()}
      }},
      {"limits", {
        {"max_linear_features", max_linear},
        {"max_point_features", max_other_features},
        {"max_area_features", max_other_features},
        {"max_coordinates_per_feature", max_coordinates},
        {"skipped_linear_features", skipped_linear},
        {"skipped_point_features", skipped_point},
        {"skipped_area_features", skipped_area},
        {"truncated", truncated}
      }},
      {"note", "OSM coordinates are geographic route geometry for reasoning, not executable map-frame MoveTo waypoints."}
    };
  }

  void save_osm_context_json(const json& osm_context) const
  {
    const std::string metadata_path = build_timestamped_path("osm_context", "json");
    std::ofstream metadata_stream(metadata_path);
    if (!metadata_stream.is_open()) {
      RCLCPP_WARN(get_logger(), "Failed to open OSM context JSON for writing: %s", metadata_path.c_str());
      return;
    }

    metadata_stream << osm_context.dump(2);
    if (!metadata_stream.good()) {
      RCLCPP_WARN(get_logger(), "Failed to write OSM context JSON: %s", metadata_path.c_str());
      return;
    }
  }

  static void add_unique_detail_point(std::vector<GeoPoint>& points, const GeoPoint& candidate)
  {
    for (const auto& existing : points) {
      if (haversine_distance_m(existing.lat, existing.lon, candidate.lat, candidate.lon) < 80.0) {
        return;
      }
    }
    points.push_back(candidate);
  }

  void apply_overpass_context_to_plan_inputs(
    const json& overpass,
    const std::string& mission_text,
    GeoBounds& mission_bounds,
    std::vector<GeoPoint>& detail_points,
    json& markers) const
  {
    if (!overpass.is_object() || !overpass.contains("elements") || !overpass["elements"].is_array()) {
      return;
    }
    const std::string text = lowercase_copy(mission_text);
    const bool area_mission = contains_any(text, {"circumvent", "around", "loop", "lake", "field", "survey", "explore", "cover", "park"});
    const bool lake_mission = contains_any(text, {"lake", "water", "shore"});
    int marker_id = 1;

    for (const auto& element : overpass["elements"]) {
      const std::string kind = classify_overpass_element(element);
      if (kind.empty()) {
        continue;
      }
      const bool include_for_extent = area_mission ||
        kind == "route" || kind == "barrier" || kind == "steps" ||
        (lake_mission && kind == "water");
      if (include_for_extent) {
        include_element_geometry(element, mission_bounds);
      }

      const auto center = element_center_point(element);
      if (!center.has_value()) {
        continue;
      }
      GeoPoint point = center.value();
      point.kind = kind;
      point.reason = "OSM " + kind + " feature";
      if (kind == "barrier" || kind == "steps" || kind == "route") {
        add_unique_detail_point(detail_points, point);
      }
      if (marker_id <= 20 && (kind == "barrier" || kind == "steps" || kind == "water" || kind == "landmark")) {
        json marker = marker_json_from_point(point, marker_id++);
        enrich_marker_from_overpass_element(marker, element, kind);
        markers.push_back(marker);
      }
    }
  }

  static TileBounds tile_bounds_for_geo_bounds(const GeoBounds& bounds, int zoom)
  {
    const int max_index = (1 << zoom) - 1;
    TileBounds tile_bounds;
    tile_bounds.min_x = std::max(0, std::min(max_index, longitude_to_tile_x(bounds.west, zoom)));
    tile_bounds.max_x = std::max(0, std::min(max_index, longitude_to_tile_x(bounds.east, zoom)));
    tile_bounds.min_y = std::max(0, std::min(max_index, latitude_to_tile_y(bounds.north, zoom)));
    tile_bounds.max_y = std::max(0, std::min(max_index, latitude_to_tile_y(bounds.south, zoom)));
    if (tile_bounds.max_x < tile_bounds.min_x) {
      std::swap(tile_bounds.max_x, tile_bounds.min_x);
    }
    if (tile_bounds.max_y < tile_bounds.min_y) {
      std::swap(tile_bounds.max_y, tile_bounds.min_y);
    }
    return tile_bounds;
  }

  TileBounds choose_overview_tile_bounds(const GeoBounds& bounds, int& zoom) const
  {
    const int min_zoom = std::max(0, std::min(22, maptiler_overview_min_zoom_));
    const int max_zoom = std::max(min_zoom, std::min(22, maptiler_overview_max_zoom_));
    const int max_tiles_per_axis = std::max(1, 2 * std::max(0, maptiler_max_overview_tile_radius_) + 1);

    TileBounds selected = tile_bounds_for_geo_bounds(bounds, min_zoom);
    zoom = min_zoom;
    for (int candidate_zoom = max_zoom; candidate_zoom >= min_zoom; --candidate_zoom) {
      const TileBounds candidate = tile_bounds_for_geo_bounds(bounds, candidate_zoom);
      const int tiles_x = candidate.max_x - candidate.min_x + 1;
      const int tiles_y = candidate.max_y - candidate.min_y + 1;
      if (tiles_x <= max_tiles_per_axis && tiles_y <= max_tiles_per_axis) {
        selected = candidate;
        zoom = candidate_zoom;
        break;
      }
    }
    return selected;
  }

  static GeoBounds bounds_from_tile_bounds(const TileBounds& tile_bounds, int zoom)
  {
    GeoBounds bounds;
    bounds.include(tile_y_to_latitude(tile_bounds.min_y, zoom), tile_x_to_longitude(tile_bounds.min_x, zoom));
    bounds.include(tile_y_to_latitude(tile_bounds.max_y + 1, zoom), tile_x_to_longitude(tile_bounds.max_x + 1, zoom));
    return bounds;
  }

  static double estimate_request_meters_per_px(const GeoBounds& bounds, const TileBounds& tile_bounds)
  {
    const int tiles_x = std::max(1, tile_bounds.max_x - tile_bounds.min_x + 1);
    const double center_lat = (bounds.north + bounds.south) / 2.0;
    const double meters = haversine_distance_m(center_lat, bounds.west, center_lat, bounds.east);
    return meters / static_cast<double>(tiles_x * 256);
  }

  json build_request_from_tile_bounds(
    const json& base_request,
    const std::string& role,
    const std::string& reason,
    int zoom,
    const TileBounds& tile_bounds) const
  {
    json request = base_request;
    const GeoBounds bounds = bounds_from_tile_bounds(tile_bounds, zoom);
    request["role"] = role;
    request["reason"] = reason;
    request["zoom"] = zoom;
    request["source"] = "maptiler";
    request["tile_bounds"] = {
      {"min_x", tile_bounds.min_x},
      {"max_x", tile_bounds.max_x},
      {"min_y", tile_bounds.min_y},
      {"max_y", tile_bounds.max_y}
    };
    request["bounds"] = {
      {"north", bounds.north},
      {"south", bounds.south},
      {"east", bounds.east},
      {"west", bounds.west}
    };
    request["center"] = {
      {"lat", (bounds.north + bounds.south) / 2.0},
      {"lon", (bounds.east + bounds.west) / 2.0}
    };
    return request;
  }

  json build_request_from_center(
    const json& base_request,
    const std::string& role,
    const std::string& reason,
    const GeoPoint& center,
    int zoom,
    int radius) const
  {
    json request = base_request;
    request["role"] = role;
    request["reason"] = reason;
    request["zoom"] = std::max(0, std::min(22, zoom));
    request["tile_radius"] = std::max(0, std::min(3, radius));
    request["center"] = {{"lat", center.lat}, {"lon", center.lon}};
    return request;
  }

  std::vector<PlannedMapArtifact> build_dynamic_satellite_plan(
    json& context_json,
    const json& satellite_request)
  {
    std::vector<PlannedMapArtifact> plan;
    double center_lat = 0.0;
    double center_lon = 0.0;
    if (!resolve_satellite_center(context_json, satellite_request, center_lat, center_lon)) {
      return plan;
    }

    const std::string mission_text = extract_mission_text(context_json);
    const double radius_m = mission_radius_m(mission_text, satellite_request);
    GeoBounds mission_bounds;
    if (!extract_bounds_from_json(satellite_request, mission_bounds)) {
      mission_bounds = bounds_from_center_radius(center_lat, center_lon, radius_m);
    }
    mission_bounds.include(center_lat, center_lon);

    json markers = json::array();
    std::vector<GeoPoint> detail_points;
    add_unique_detail_point(
      detail_points,
      GeoPoint{center_lat, center_lon, "Robot", "start", "Robot/start detail map"});

    const double overpass_radius = std::max(radius_m, overpass_radius_m_);
    const json overpass = get_or_fetch_overpass_context(context_json, center_lat, center_lon, overpass_radius);
    apply_overpass_context_to_plan_inputs(overpass, mission_text, mission_bounds, detail_points, markers);

    const std::string lower_text = lowercase_copy(mission_text);
    const double padding_m = contains_any(lower_text, {"circumvent", "around", "loop", "lake", "field", "survey", "explore", "cover"})
      ? std::max(100.0, radius_m * 0.15)
      : std::max(75.0, radius_m * 0.25);
    mission_bounds = pad_bounds_m(mission_bounds, padding_m);
    if (!mission_bounds.valid()) {
      mission_bounds = bounds_from_center_radius(center_lat, center_lon, radius_m);
    }

    int overview_zoom = maptiler_tile_zoom_;
    const TileBounds overview_tiles = choose_overview_tile_bounds(mission_bounds, overview_zoom);
    json overview_request = build_request_from_tile_bounds(
      satellite_request,
      "overview",
      "Full deterministic mission extent",
      overview_zoom,
      overview_tiles);
    overview_request["robot_location"] = {{"lat", center_lat}, {"lon", center_lon}, {"label", "Robot"}};
    if (!markers.empty()) {
      overview_request["points_of_interest"] = markers;
    }
    if (!overpass.empty()) {
      overview_request["osm_context"] = {
        {"provider", "overpass"},
        {"radius_m", overpass_radius},
        {"element_count", overpass.value("elements", json::array()).size()}
      };
    }
    plan.push_back(PlannedMapArtifact{"overview", "Full deterministic mission extent", overview_request});

    const GeoBounds overview_bounds = bounds_from_tile_bounds(overview_tiles, overview_zoom);
    const double overview_m_per_px = estimate_request_meters_per_px(overview_bounds, overview_tiles);
    const bool needs_detail = overview_m_per_px > maptiler_overview_target_max_m_per_px_ ||
      detail_points.size() > 1;
    const int max_artifacts = std::max(1, maptiler_max_map_artifacts_);
    if (!needs_detail || max_artifacts <= 1) {
      return plan;
    }

    int detail_index = 0;
    for (const auto& point : detail_points) {
      if (static_cast<int>(plan.size()) >= max_artifacts) {
        break;
      }
      const std::string role = detail_index == 0 ? "detail_start" : "detail_route_" + std::to_string(detail_index);
      const std::string reason = point.reason.empty() ? "High-zoom OSM detail" : point.reason;
      json detail_request = build_request_from_center(
        satellite_request,
        role,
        reason,
        point,
        maptiler_detail_zoom_,
        1);
      detail_request["robot_location"] = {{"lat", center_lat}, {"lon", center_lon}, {"label", "Robot"}};
      if (!markers.empty()) {
        detail_request["points_of_interest"] = markers;
      }
      plan.push_back(PlannedMapArtifact{role, reason, detail_request});
      ++detail_index;
    }
    return plan;
  }

  static std::string normalize_maptiler_api_mode(std::string mode)
  {
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (mode == "map" || mode == "maps") {
      return "maps";
    }
    return "tiles";
  }

  std::string resolve_maptiler_api_mode(const json& satellite_request) const
  {
    if (satellite_request.is_object()) {
      const auto request_mode = extract_first_string(
        satellite_request,
        {"maptiler_api_mode", "maptiler_mode", "maptiler_endpoint_kind", "maptiler_resource_kind"});
      if (!request_mode.empty()) {
        return normalize_maptiler_api_mode(request_mode);
      }
    }
    return normalize_maptiler_api_mode(maptiler_api_mode_);
  }

  std::string build_maptiler_tile_url(
    const std::string& api_mode,
    int zoom,
    int tile_x,
    int tile_y) const
  {
    if (api_mode == "maps") {
      return "https://api.maptiler.com/maps/" + maptiler_style_id_ + "/256/" +
        std::to_string(zoom) + "/" + std::to_string(tile_x) + "/" + std::to_string(tile_y) +
        ".png?key=" + maptiler_api_key_;
    }
    return "https://api.maptiler.com/tiles/" + maptiler_style_id_ + "/" +
      std::to_string(zoom) + "/" + std::to_string(tile_x) + "/" + std::to_string(tile_y) +
      "?key=" + maptiler_api_key_;
  }

  static double clamp_latitude(double latitude)
  {
    return std::max(-85.05112878, std::min(85.05112878, latitude));
  }

  static double normalize_longitude(double longitude)
  {
    while (longitude < -180.0) {
      longitude += 360.0;
    }
    while (longitude > 180.0) {
      longitude -= 360.0;
    }
    return longitude;
  }

  static int longitude_to_tile_x(double longitude, int zoom)
  {
    const double n = std::pow(2.0, static_cast<double>(zoom));
    const double normalized = (normalize_longitude(longitude) + 180.0) / 360.0;
    return static_cast<int>(std::floor(normalized * n));
  }

  static int latitude_to_tile_y(double latitude, int zoom)
  {
    const double clamped = clamp_latitude(latitude);
    const double lat_rad = clamped * pi() / 180.0;
    const double n = std::pow(2.0, static_cast<double>(zoom));
    const double y = (1.0 - std::asinh(std::tan(lat_rad)) / pi()) / 2.0 * n;
    return static_cast<int>(std::floor(y));
  }

  static double tile_x_to_longitude(int x, int zoom)
  {
    const double n = std::pow(2.0, static_cast<double>(zoom));
    return static_cast<double>(x) / n * 360.0 - 180.0;
  }

  static double tile_y_to_latitude(int y, int zoom)
  {
    const double n = std::pow(2.0, static_cast<double>(zoom));
    const double mercator = pi() * (1.0 - 2.0 * static_cast<double>(y) / n);
    return std::atan(std::sinh(mercator)) * 180.0 / pi();
  }

  static double haversine_distance_m(double lat1, double lon1, double lat2, double lon2)
  {
    constexpr double earth_radius_m = 6371000.0;
    const double deg_to_rad = pi() / 180.0;
    const double lat1_rad = lat1 * deg_to_rad;
    const double lat2_rad = lat2 * deg_to_rad;
    const double delta_lat = (lat2 - lat1) * deg_to_rad;
    const double delta_lon = (lon2 - lon1) * deg_to_rad;
    const double a = std::sin(delta_lat / 2.0) * std::sin(delta_lat / 2.0) +
      std::cos(lat1_rad) * std::cos(lat2_rad) *
      std::sin(delta_lon / 2.0) * std::sin(delta_lon / 2.0);
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return earth_radius_m * c;
  }

  static bool extract_tile_bounds(const json& request, TileBounds& tile_bounds)
  {
    if (!request.is_object() || !request.contains("tile_bounds") || !request["tile_bounds"].is_object()) {
      return false;
    }
    const auto& bounds = request["tile_bounds"];
    tile_bounds.min_x = extract_int_from_json(bounds, "min_x", 0);
    tile_bounds.max_x = extract_int_from_json(bounds, "max_x", tile_bounds.min_x);
    tile_bounds.min_y = extract_int_from_json(bounds, "min_y", 0);
    tile_bounds.max_y = extract_int_from_json(bounds, "max_y", tile_bounds.min_y);
    if (tile_bounds.max_x < tile_bounds.min_x) {
      std::swap(tile_bounds.max_x, tile_bounds.min_x);
    }
    if (tile_bounds.max_y < tile_bounds.min_y) {
      std::swap(tile_bounds.max_y, tile_bounds.min_y);
    }
    return true;
  }

  static bool extract_lat_lon_from_entry(const json& entry, double& latitude, double& longitude)
  {
    const auto lat = extract_number_from_json(entry, {"latitude", "lat"});
    const auto lon = extract_number_from_json(entry, {"longitude", "lon", "lng"});
    if (!lat.has_value() || !lon.has_value()) {
      return false;
    }
    latitude = lat.value();
    longitude = lon.value();
    return std::isfinite(latitude) && std::isfinite(longitude);
  }

  bool resolve_satellite_center(
    const json& context_json,
    const json& satellite_request,
    double& latitude,
    double& longitude)
  {
    if (satellite_request.is_object()) {
      const auto robot_location = satellite_request.find("robot_location");
      if (robot_location != satellite_request.end() &&
          extract_lat_lon_from_entry(*robot_location, latitude, longitude))
      {
        return true;
      }

      const auto center = satellite_request.find("center");
      if (center != satellite_request.end() &&
          extract_lat_lon_from_entry(*center, latitude, longitude))
      {
        return true;
      }

      const auto bounds_it = satellite_request.find("bounds");
      if (bounds_it != satellite_request.end() && bounds_it->is_object()) {
        const auto north = extract_number_from_json(*bounds_it, {"north", "max_lat"});
        const auto south = extract_number_from_json(*bounds_it, {"south", "min_lat"});
        const auto east = extract_number_from_json(*bounds_it, {"east", "max_lon"});
        const auto west = extract_number_from_json(*bounds_it, {"west", "min_lon"});
        if (north.has_value() && south.has_value() && east.has_value() && west.has_value()) {
          latitude = (north.value() + south.value()) / 2.0;
          longitude = (east.value() + west.value()) / 2.0;
          return true;
        }
      }
    }

    const auto gps_fix = context_json.find("GPS_FIX");
    if (gps_fix != context_json.end() && gps_fix->is_object() &&
        extract_lat_lon_from_entry(*gps_fix, latitude, longitude))
    {
      return true;
    }

    const GpsSnapshot latest_gps = get_latest_gps_snapshot();
    if (latest_gps.available) {
      latitude = latest_gps.latitude;
      longitude = latest_gps.longitude;
      return true;
    }
    return false;
  }

  bool download_binary_url(const std::string& url, std::vector<unsigned char>& buffer, std::string& error_message)
  {
    if (!curl_ready_) {
      error_message = "libcurl is not initialized";
      return false;
    }

    buffer.clear();
    CURL* curl = curl_easy_init();
    if (!curl) {
      error_message = "curl_easy_init failed";
      return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &ContextGathererNode::curl_write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "generalist_bt_gen/context_gatherer");
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(maptiler_request_timeout_sec_ * 1000.0));
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    const CURLcode rc = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);

    if (rc != CURLE_OK) {
      error_message = curl_easy_strerror(rc);
      return false;
    }
    if (http_code < 200 || http_code >= 300) {
      error_message = "HTTP " + std::to_string(http_code);
      return false;
    }
    return true;
  }

  SatelliteFetchResult fetch_satellite_map_from_maptiler(
    const json& context_json,
    const json& satellite_request)
  {
    SatelliteFetchResult result;
    if (maptiler_api_key_.empty()) {
      result.message = "MAPTILER_API_KEY / maptiler_api_key is not configured";
      return result;
    }

    double center_lat = 0.0;
    double center_lon = 0.0;
    if (!resolve_satellite_center(context_json, satellite_request, center_lat, center_lon)) {
      result.message = "No GPS or satellite request center available";
      return result;
    }

    const int zoom = std::max(0, std::min(22, extract_int_from_json(
      satellite_request, "zoom", maptiler_tile_zoom_)));
    const std::string api_mode = resolve_maptiler_api_mode(satellite_request);
    const int max_index = (1 << zoom) - 1;

    TileBounds requested_tiles;
    const bool has_tile_bounds = extract_tile_bounds(satellite_request, requested_tiles);
    const int radius = std::max(0, std::min(3, extract_int_from_json(
      satellite_request, "tile_radius", maptiler_tile_radius_)));
    int min_x = 0;
    int max_x = 0;
    int min_y = 0;
    int max_y = 0;
    if (has_tile_bounds) {
      min_x = std::max(0, std::min(max_index, requested_tiles.min_x));
      max_x = std::max(0, std::min(max_index, requested_tiles.max_x));
      min_y = std::max(0, std::min(max_index, requested_tiles.min_y));
      max_y = std::max(0, std::min(max_index, requested_tiles.max_y));
    } else {
      const int center_x = std::max(0, std::min(max_index, longitude_to_tile_x(center_lon, zoom)));
      const int center_y = std::max(0, std::min(max_index, latitude_to_tile_y(center_lat, zoom)));
      min_x = std::max(0, center_x - radius);
      max_x = std::min(max_index, center_x + radius);
      min_y = std::max(0, center_y - radius);
      max_y = std::min(max_index, center_y + radius);
    }
    if (max_x < min_x) {
      std::swap(max_x, min_x);
    }
    if (max_y < min_y) {
      std::swap(max_y, min_y);
    }

    cv::Mat stitched;
    int tile_width = 0;
    int tile_height = 0;

    for (int tile_y = min_y; tile_y <= max_y; ++tile_y) {
      std::vector<cv::Mat> row_tiles;
      for (int tile_x = min_x; tile_x <= max_x; ++tile_x) {
        const std::string url = build_maptiler_tile_url(api_mode, zoom, tile_x, tile_y);

        std::vector<unsigned char> encoded_tile;
        std::string error_message;
        if (!download_binary_url(url, encoded_tile, error_message)) {
          result.message = "Failed to download MapTiler tile " +
            std::to_string(tile_x) + "," + std::to_string(tile_y) + ": " + error_message;
          return result;
        }

        cv::Mat decoded = cv::imdecode(encoded_tile, cv::IMREAD_COLOR);
        if (decoded.empty()) {
          result.message = "Failed to decode MapTiler tile image";
          return result;
        }

        if (tile_width == 0) {
          tile_width = decoded.cols;
          tile_height = decoded.rows;
        } else if (decoded.cols != tile_width || decoded.rows != tile_height) {
          result.message = "MapTiler tiles returned inconsistent dimensions";
          return result;
        }
        row_tiles.push_back(decoded);
      }

      cv::Mat row_image;
      cv::hconcat(row_tiles, row_image);
      if (stitched.empty()) {
        stitched = row_image;
      } else {
        cv::vconcat(stitched, row_image, stitched);
      }
    }

    if (stitched.empty()) {
      result.message = "No satellite tiles were stitched";
      return result;
    }

    const double west = tile_x_to_longitude(min_x, zoom);
    const double east = tile_x_to_longitude(max_x + 1, zoom);
    const double north = tile_y_to_latitude(min_y, zoom);
    const double south = tile_y_to_latitude(max_y + 1, zoom);
    const std::string session_id = extract_mission_session_id(context_json);
    const std::string safe_session = session_id.empty() ? "unknown" : session_id;
    const std::string image_path = build_timestamped_path("maptiler_satellite_" + safe_session, "png");
    if (!cv::imwrite(image_path, stitched)) {
      result.message = "Failed to write stitched satellite image";
      return result;
    }

    json request_json = satellite_request;
    request_json["source"] = "maptiler";
    request_json["maptiler_api_mode"] = api_mode;
    request_json["source_image_uri"] = to_file_uri(image_path);
    request_json["zoom"] = zoom;
    if (!has_tile_bounds) {
      request_json["tile_radius"] = radius;
    }
    request_json["tile_bounds"] = {
      {"min_x", min_x},
      {"max_x", max_x},
      {"min_y", min_y},
      {"max_y", max_y}
    };
    request_json["bounds"] = {
      {"north", north},
      {"south", south},
      {"east", east},
      {"west", west}
    };
    if (!request_json.contains("robot_location")) {
      request_json["robot_location"] = {
        {"lat", center_lat},
        {"lon", center_lon},
        {"label", "Robot"}
      };
    }
    request_json["center"] = {
      {"lat", center_lat},
      {"lon", center_lon}
    };

    result.fetch_metadata = {
      {"provider", "maptiler"},
      {"api_mode", api_mode},
      {"style_id", maptiler_style_id_},
      {"zoom", zoom},
      {"tile_radius", has_tile_bounds ? json(nullptr) : json(radius)},
      {"tile_x_min", min_x},
      {"tile_x_max", max_x},
      {"tile_y_min", min_y},
      {"tile_y_max", max_y},
      {"center_lat", center_lat},
      {"center_lon", center_lon},
      {"image_width_px", stitched.cols},
      {"image_height_px", stitched.rows}
    };
    result.image_uri = to_file_uri(image_path);
    result.request_json = request_json;
    result.success = true;
    result.message = "Fetched satellite map from MapTiler";
    return result;
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

  static std::string sanitize_path_token(const std::string& raw)
  {
    std::string sanitized;
    for (const char c : raw) {
      if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
        sanitized.push_back(c);
      } else {
        sanitized.push_back('_');
      }
    }
    return sanitized.empty() ? "unknown" : sanitized;
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

    context_json.erase("__OVERPASS_CACHE");
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
