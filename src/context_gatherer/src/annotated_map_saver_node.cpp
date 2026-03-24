#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <array>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

class AnnotatedMapSaverNode : public rclcpp::Node
{
public:
  using SaveMap = nav2_msgs::srv::SaveMap;

  AnnotatedMapSaverNode()
  : Node("annotated_map_saver")
  {
    output_directory_ = this->declare_parameter<std::string>("output_directory", "/tmp/context_gatherer");
    pose_cov_topic_ = this->declare_parameter<std::string>("pose_cov_topic", "");
    slam_map_topic_ = this->declare_parameter<std::string>("slam_map_topic", "/map");
    annotated_map_service_name_ = this->declare_parameter<std::string>(
      "annotated_map_service_name", "/annotated_map_saver/save_map");
    annotated_map_grid_spacing_m_ = this->declare_parameter<double>(
      "annotated_map_grid_spacing_m", 0.0);
    debug_logging_ = this->declare_parameter<bool>("enable_debug_logging", true);

    std::filesystem::create_directories(output_directory_);

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
    slam_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      slam_map_topic_, 1,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_slam_map_ = msg;
      });

    save_map_service_ = create_service<SaveMap>(
      annotated_map_service_name_,
      std::bind(
        &AnnotatedMapSaverNode::handle_save_annotated_map, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Annotated map saver ready on %s (map_topic=%s)",
      annotated_map_service_name_.c_str(),
      slam_map_topic_.c_str());
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
    std::string metadata_path;
    double grid_spacing_m{0.0};
    bool robot_drawn{false};
  };

  std::string output_directory_;
  std::string pose_cov_topic_;
  std::string slam_map_topic_;
  std::string annotated_map_service_name_;
  double annotated_map_grid_spacing_m_;
  bool debug_logging_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slam_map_sub_;
  rclcpp::Service<SaveMap>::SharedPtr save_map_service_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_cov_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_slam_map_;
  std::mutex data_mutex_;

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

  nav_msgs::msg::OccupancyGrid::SharedPtr get_latest_slam_map()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_slam_map_;
  }

  static std::string to_lower_copy(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value;
  }

  static std::string format_decimal(double value, int precision = 2)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  std::string build_timestamped_path(const std::string& prefix, const std::string& extension) const
  {
    const auto stamp_ns = this->now().nanoseconds();
    return output_directory_ + "/" + prefix + "_" + std::to_string(stamp_ns) + "." + extension;
  }

  static std::string strip_file_uri(const std::string& uri)
  {
    constexpr const char* prefix = "file://";
    if (uri.rfind(prefix, 0) == 0) {
      return uri.substr(std::strlen(prefix));
    }
    return uri;
  }

  static bool is_yaml_extension(const std::string& extension)
  {
    return extension == ".yaml" || extension == ".yml";
  }

  static bool is_image_extension(const std::string& extension)
  {
    return extension == ".png" || extension == ".pgm" || extension == ".bmp" ||
           extension == ".jpg" || extension == ".jpeg";
  }

  std::string normalize_image_format(std::string image_format) const
  {
    image_format = to_lower_copy(image_format);
    if (image_format.empty()) {
      return "png";
    }
    if (image_format == "jpeg") {
      return "jpg";
    }
    if (image_format == "jpg" || image_format == "png" || image_format == "bmp" ||
        image_format == "pgm")
    {
      return image_format;
    }
    return "png";
  }

  std::string normalize_map_mode(std::string map_mode) const
  {
    map_mode = to_lower_copy(map_mode);
    if (map_mode == "scale" || map_mode == "raw") {
      return map_mode;
    }
    return "trinary";
  }

  bool resolve_annotated_map_paths(
    const std::string& requested_map_url,
    const std::string& requested_image_format,
    AnnotatedMapArtifacts& artifacts,
    std::string& error) const
  {
    std::string path_input = strip_file_uri(requested_map_url);
    if (path_input.rfind("package://", 0) == 0) {
      error = "package:// map_url values are not supported by the annotated saver";
      return false;
    }
    if (path_input.empty()) {
      path_input = build_timestamped_path("annotated_slam_map", "yaml");
    }

    const std::string image_format = normalize_image_format(requested_image_format);
    std::filesystem::path base_path(path_input);
    const std::string extension = to_lower_copy(base_path.extension().string());

    if (is_yaml_extension(extension)) {
      artifacts.metadata_path = base_path.string();
      artifacts.image_path = base_path.replace_extension("." + image_format).string();
    } else if (is_image_extension(extension)) {
      artifacts.image_path = base_path.string();
      artifacts.metadata_path = base_path.replace_extension(".yaml").string();
    } else {
      artifacts.image_path = base_path.string() + "." + image_format;
      artifacts.metadata_path = base_path.string() + ".yaml";
    }

    const std::filesystem::path image_parent = std::filesystem::path(artifacts.image_path).parent_path();
    const std::filesystem::path metadata_parent = std::filesystem::path(artifacts.metadata_path).parent_path();
    if (!image_parent.empty()) {
      std::filesystem::create_directories(image_parent);
    }
    if (!metadata_parent.empty()) {
      std::filesystem::create_directories(metadata_parent);
    }

    return true;
  }

  static cv::Scalar occupancy_to_color(
    const int8_t value,
    const std::string& map_mode,
    const float free_thresh,
    const float occupied_thresh)
  {
    if (value < 0) {
      return cv::Scalar(160, 160, 160);
    }

    const int clamped = std::max(0, std::min(100, static_cast<int>(value)));
    const double probability = static_cast<double>(clamped) / 100.0;
    if (map_mode == "trinary") {
      if (probability >= occupied_thresh) {
        return cv::Scalar(0, 0, 0);
      }
      if (probability <= free_thresh) {
        return cv::Scalar(255, 255, 255);
      }
      return cv::Scalar(205, 205, 205);
    }

    const int intensity = 255 - (clamped * 255) / 100;
    return cv::Scalar(intensity, intensity, intensity);
  }

  double choose_grid_spacing_m(double resolution) const
  {
    if (annotated_map_grid_spacing_m_ > 0.0) {
      return annotated_map_grid_spacing_m_;
    }

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

  static double choose_scale_bar_length_m(double resolution)
  {
    const double target_length_m = std::max(resolution, resolution * 120.0);
    const double magnitude = std::pow(10.0, std::floor(std::log10(target_length_m)));
    for (double factor : {1.0, 2.0, 5.0, 10.0}) {
      const double candidate = factor * magnitude;
      if (candidate >= target_length_m) {
        return candidate;
      }
    }
    return magnitude * 10.0;
  }

  static bool world_to_image(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const double world_x,
    const double world_y,
    cv::Point& point)
  {
    const double yaw = quaternion_to_yaw(map->info.origin.orientation);
    const double dx = world_x - map->info.origin.position.x;
    const double dy = world_y - map->info.origin.position.y;
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double grid_x = local_x / map->info.resolution;
    const double grid_y = local_y / map->info.resolution;

    const int pixel_x = static_cast<int>(std::round(grid_x));
    const int pixel_y =
      static_cast<int>(map->info.height) - 1 - static_cast<int>(std::round(grid_y));
    point = cv::Point(pixel_x, pixel_y);

    return pixel_x >= 0 && pixel_x < static_cast<int>(map->info.width) &&
           pixel_y >= 0 && pixel_y < static_cast<int>(map->info.height);
  }

  static cv::Point2d world_to_image_point(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const double world_x,
    const double world_y)
  {
    const double yaw = quaternion_to_yaw(map->info.origin.orientation);
    const double dx = world_x - map->info.origin.position.x;
    const double dy = world_y - map->info.origin.position.y;
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    return cv::Point2d(
      local_x / map->info.resolution,
      static_cast<double>(map->info.height - 1) - (local_y / map->info.resolution));
  }

  static cv::Point2d world_vector_to_image_delta(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const double world_dx,
    const double world_dy)
  {
    const double yaw = quaternion_to_yaw(map->info.origin.orientation);
    const double local_x = std::cos(yaw) * world_dx + std::sin(yaw) * world_dy;
    const double local_y = -std::sin(yaw) * world_dx + std::cos(yaw) * world_dy;
    return cv::Point2d(
      local_x / map->info.resolution,
      -(local_y / map->info.resolution));
  }

  static cv::Point map_vector_to_image_vector(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const double vector_x,
    const double vector_y,
    const double scale_px)
  {
    const double yaw = quaternion_to_yaw(map->info.origin.orientation);
    const double local_x = std::cos(yaw) * vector_x + std::sin(yaw) * vector_y;
    const double local_y = -std::sin(yaw) * vector_x + std::cos(yaw) * vector_y;
    const double magnitude = std::hypot(local_x, local_y);
    if (magnitude < 1e-6) {
      return cv::Point(0, 0);
    }

    return cv::Point(
      static_cast<int>(std::round((local_x / magnitude) * scale_px)),
      static_cast<int>(std::round((-local_y / magnitude) * scale_px)));
  }

  static void draw_axis_widget(
    cv::Mat& image,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
  {
    const cv::Point origin(image.cols - 90, 90);
    const cv::Point x_arrow = origin + map_vector_to_image_vector(map, 1.0, 0.0, 50.0);
    const cv::Point y_arrow = origin + map_vector_to_image_vector(map, 0.0, 1.0, 50.0);

    cv::circle(image, origin, 4, cv::Scalar(50, 50, 50), cv::FILLED);
    cv::arrowedLine(image, origin, x_arrow, cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.15);
    cv::arrowedLine(image, origin, y_arrow, cv::Scalar(0, 180, 0), 2, cv::LINE_AA, 0, 0.15);
    cv::putText(
      image, "+X", x_arrow + cv::Point(4, -4), cv::FONT_HERSHEY_SIMPLEX, 0.45,
      cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::putText(
      image, "+Y", y_arrow + cv::Point(4, -4), cv::FONT_HERSHEY_SIMPLEX, 0.45,
      cv::Scalar(0, 150, 0), 1, cv::LINE_AA);
    cv::putText(
      image, "map frame", origin + cv::Point(-34, 22), cv::FONT_HERSHEY_SIMPLEX, 0.45,
      cv::Scalar(40, 40, 40), 1, cv::LINE_AA);
  }

  static void draw_metric_grid(
    cv::Mat& image,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const double grid_spacing_m)
  {
    const int width = static_cast<int>(map->info.width);
    const int height = static_cast<int>(map->info.height);
    const cv::Rect image_rect(0, 0, width, height);
    const int label_every = std::max(1, static_cast<int>(std::ceil(5.0 / grid_spacing_m)));
    const int label_precision = grid_spacing_m >= 1.0 ? 0 : (grid_spacing_m >= 0.1 ? 1 : 2);
    const double line_extent = static_cast<double>(std::max(width, height)) * 2.0;

    const double yaw = quaternion_to_yaw(map->info.origin.orientation);
    const double width_m = static_cast<double>(map->info.width) * map->info.resolution;
    const double height_m = static_cast<double>(map->info.height) * map->info.resolution;
    const std::array<cv::Point2d, 4> corners{
      cv::Point2d(map->info.origin.position.x, map->info.origin.position.y),
      cv::Point2d(
        map->info.origin.position.x + std::cos(yaw) * width_m,
        map->info.origin.position.y + std::sin(yaw) * width_m),
      cv::Point2d(
        map->info.origin.position.x - std::sin(yaw) * height_m,
        map->info.origin.position.y + std::cos(yaw) * height_m),
      cv::Point2d(
        map->info.origin.position.x + std::cos(yaw) * width_m - std::sin(yaw) * height_m,
        map->info.origin.position.y + std::sin(yaw) * width_m + std::cos(yaw) * height_m)
    };

    double min_world_x = corners[0].x;
    double max_world_x = corners[0].x;
    double min_world_y = corners[0].y;
    double max_world_y = corners[0].y;
    for (const auto& corner : corners) {
      min_world_x = std::min(min_world_x, corner.x);
      max_world_x = std::max(max_world_x, corner.x);
      min_world_y = std::min(min_world_y, corner.y);
      max_world_y = std::max(max_world_y, corner.y);
    }

    const cv::Point2d x_line_direction = world_vector_to_image_delta(map, 0.0, 1.0);
    const cv::Point2d y_line_direction = world_vector_to_image_delta(map, 1.0, 0.0);
    const double x_line_norm = std::hypot(x_line_direction.x, x_line_direction.y);
    const double y_line_norm = std::hypot(y_line_direction.x, y_line_direction.y);
    if (x_line_norm < 1e-6 || y_line_norm < 1e-6) {
      return;
    }

    const cv::Point2d x_line_unit = x_line_direction * (1.0 / x_line_norm);
    const cv::Point2d y_line_unit = y_line_direction * (1.0 / y_line_norm);

    auto format_signed_metric = [label_precision](double value) {
      if (std::abs(value) < 1e-6) {
        return std::string("0m");
      }
      const std::string prefix = value > 0.0 ? "+" : "-";
      return prefix + format_decimal(std::abs(value), label_precision) + "m";
    };

    for (int grid_index = static_cast<int>(std::floor(min_world_x / grid_spacing_m));
         grid_index <= static_cast<int>(std::ceil(max_world_x / grid_spacing_m));
         ++grid_index)
    {
      const double world_x = static_cast<double>(grid_index) * grid_spacing_m;
      const bool zero_line = std::abs(world_x) < 1e-6;
      const bool major = zero_line || (std::abs(grid_index) % label_every) == 0;
      const cv::Point2d anchor = world_to_image_point(map, world_x, 0.0);
      cv::Point p1(
        static_cast<int>(std::round(anchor.x - x_line_unit.x * line_extent)),
        static_cast<int>(std::round(anchor.y - x_line_unit.y * line_extent)));
      cv::Point p2(
        static_cast<int>(std::round(anchor.x + x_line_unit.x * line_extent)),
        static_cast<int>(std::round(anchor.y + x_line_unit.y * line_extent)));
      if (!cv::clipLine(image_rect, p1, p2)) {
        continue;
      }

      const cv::Scalar color = zero_line ? cv::Scalar(180, 210, 255)
        : (major ? cv::Scalar(210, 220, 255) : cv::Scalar(225, 235, 255));
      cv::line(image, p1, p2, color, zero_line ? 2 : 1, cv::LINE_AA);
      if (major) {
        const cv::Point label_anchor = p1.y > p2.y ? p1 : p2;
        const cv::Point label_position(
          std::clamp(label_anchor.x + 4, 4, width - 52),
          std::clamp(label_anchor.y - 6, 14, height - 6));
        cv::putText(
          image,
          format_signed_metric(world_x),
          label_position,
          cv::FONT_HERSHEY_SIMPLEX,
          0.35,
          zero_line ? cv::Scalar(60, 80, 140) : cv::Scalar(70, 70, 70),
          1,
          cv::LINE_AA);
      }
    }

    for (int grid_index = static_cast<int>(std::floor(min_world_y / grid_spacing_m));
         grid_index <= static_cast<int>(std::ceil(max_world_y / grid_spacing_m));
         ++grid_index)
    {
      const double world_y = static_cast<double>(grid_index) * grid_spacing_m;
      const bool zero_line = std::abs(world_y) < 1e-6;
      const bool major = zero_line || (std::abs(grid_index) % label_every) == 0;
      const cv::Point2d anchor = world_to_image_point(map, 0.0, world_y);
      cv::Point p1(
        static_cast<int>(std::round(anchor.x - y_line_unit.x * line_extent)),
        static_cast<int>(std::round(anchor.y - y_line_unit.y * line_extent)));
      cv::Point p2(
        static_cast<int>(std::round(anchor.x + y_line_unit.x * line_extent)),
        static_cast<int>(std::round(anchor.y + y_line_unit.y * line_extent)));
      if (!cv::clipLine(image_rect, p1, p2)) {
        continue;
      }

      const cv::Scalar color = zero_line ? cv::Scalar(200, 240, 200)
        : (major ? cv::Scalar(220, 240, 220) : cv::Scalar(232, 245, 232));
      cv::line(image, p1, p2, color, zero_line ? 2 : 1, cv::LINE_AA);
      if (major) {
        const cv::Point label_anchor = p1.x < p2.x ? p1 : p2;
        const cv::Point label_position(
          std::clamp(label_anchor.x + 4, 4, width - 52),
          std::clamp(label_anchor.y - 4, 14, height - 6));
        cv::putText(
          image,
          format_signed_metric(world_y),
          label_position,
          cv::FONT_HERSHEY_SIMPLEX,
          0.35,
          zero_line ? cv::Scalar(50, 120, 50) : cv::Scalar(70, 70, 70),
          1,
          cv::LINE_AA);
      }
    }
  }

  static void draw_scale_bar(
    cv::Mat& image,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const double scale_bar_m)
  {
    const int bar_px = std::max(
      1, static_cast<int>(std::round(scale_bar_m / map->info.resolution)));
    const int y = image.rows - 24;
    const int x = std::max(12, image.cols - bar_px - 24);
    const cv::Point start(x, y);
    const cv::Point end(x + bar_px, y);

    cv::line(image, start, end, cv::Scalar(35, 35, 35), 3, cv::LINE_AA);
    cv::line(image, start + cv::Point(0, -6), start + cv::Point(0, 6), cv::Scalar(35, 35, 35), 2);
    cv::line(image, end + cv::Point(0, -6), end + cv::Point(0, 6), cv::Scalar(35, 35, 35), 2);
    cv::putText(
      image,
      format_decimal(scale_bar_m, 1) + " m",
      cv::Point(x, y - 10),
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      cv::Scalar(30, 30, 30),
      1,
      cv::LINE_AA);
  }

  static bool draw_robot_pose(
    cv::Mat& image,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const PoseSnapshot& pose)
  {
    if (!pose.available || !pose.in_map_frame) {
      return false;
    }

    cv::Point center;
    if (!world_to_image(map, pose.x, pose.y, center)) {
      return false;
    }

    const double map_yaw = quaternion_to_yaw(map->info.origin.orientation);
    const double local_heading = pose.yaw - map_yaw;
    const int arrow_length = std::max(
      12, static_cast<int>(std::round(0.75 / map->info.resolution)));
    const cv::Point tip(
      center.x + static_cast<int>(std::round(std::cos(local_heading) * arrow_length)),
      center.y - static_cast<int>(std::round(std::sin(local_heading) * arrow_length)));

    cv::circle(image, center, 6, cv::Scalar(0, 80, 255), cv::FILLED);
    cv::circle(image, center, 9, cv::Scalar(255, 255, 255), 2);
    cv::arrowedLine(image, center, tip, cv::Scalar(0, 0, 180), 2, cv::LINE_AA, 0, 0.25);
    cv::putText(
      image,
      "robot",
      center + cv::Point(10, -10),
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      cv::Scalar(10, 10, 120),
      1,
      cv::LINE_AA);
    return true;
  }

  bool render_annotated_map_image(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const PoseSnapshot& pose,
    const std::string& image_path,
    const std::string& requested_map_mode,
    const float requested_free_thresh,
    const float requested_occupied_thresh,
    AnnotatedMapArtifacts& artifacts,
    std::string& error) const
  {
    const int width = static_cast<int>(map->info.width);
    const int height = static_cast<int>(map->info.height);
    if (width <= 0 || height <= 0) {
      error = "map dimensions are invalid";
      return false;
    }

    const std::string map_mode = normalize_map_mode(requested_map_mode);
    const float free_thresh = std::clamp(requested_free_thresh, 0.0f, 1.0f);
    const float occupied_thresh = std::clamp(requested_occupied_thresh, 0.0f, 1.0f);
    cv::Mat image(height, width, CV_8UC3);
    const auto& data = map->data;

    for (int grid_y = 0; grid_y < height; ++grid_y) {
      const int image_y = height - 1 - grid_y;
      for (int x = 0; x < width; ++x) {
        const size_t idx = static_cast<size_t>(grid_y) * static_cast<size_t>(width) + x;
        const int8_t value = idx < data.size() ? data[idx] : -1;
        const cv::Scalar color = occupancy_to_color(
          value, map_mode, free_thresh, occupied_thresh);
        image.at<cv::Vec3b>(image_y, x) = cv::Vec3b(
          static_cast<uint8_t>(color[0]),
          static_cast<uint8_t>(color[1]),
          static_cast<uint8_t>(color[2]));
      }
    }

    artifacts.grid_spacing_m = choose_grid_spacing_m(map->info.resolution);
    draw_metric_grid(image, map, artifacts.grid_spacing_m);
    draw_axis_widget(image, map);
    draw_scale_bar(image, map, choose_scale_bar_length_m(map->info.resolution));
    artifacts.robot_drawn = draw_robot_pose(image, map, pose);

    try {
      cv::imwrite(image_path, image);
    } catch (const std::exception& exception) {
      error = exception.what();
      return false;
    }

    return true;
  }

  bool write_annotated_map_metadata(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const PoseSnapshot& pose,
    const std::string& requested_map_mode,
    const float requested_free_thresh,
    const float requested_occupied_thresh,
    const AnnotatedMapArtifacts& artifacts,
    std::string& error) const
  {
    const std::string map_mode = normalize_map_mode(requested_map_mode);
    const float free_thresh = std::clamp(requested_free_thresh, 0.0f, 1.0f);
    const float occupied_thresh = std::clamp(requested_occupied_thresh, 0.0f, 1.0f);
    std::ofstream metadata_stream(artifacts.metadata_path);
    if (!metadata_stream) {
      error = "failed to open metadata file for writing";
      return false;
    }

    metadata_stream << "image: " << std::filesystem::path(artifacts.image_path).filename().string() << "\n";
    metadata_stream << "mode: " << map_mode << "\n";
    metadata_stream << "resolution: " << map->info.resolution << "\n";
    metadata_stream << "origin: [" << map->info.origin.position.x << ", "
                    << map->info.origin.position.y << ", "
                    << quaternion_to_yaw(map->info.origin.orientation) << "]\n";
    metadata_stream << "negate: 0\n";
    metadata_stream << "occupied_thresh: " << occupied_thresh << "\n";
    metadata_stream << "free_thresh: " << free_thresh << "\n";
    metadata_stream << "annotated: true\n";
    metadata_stream << "grid_spacing_m: " << artifacts.grid_spacing_m << "\n";
    metadata_stream << "robot_pose_available: " << (pose.available ? "true" : "false") << "\n";
    metadata_stream << "robot_pose_drawn: " << (artifacts.robot_drawn ? "true" : "false") << "\n";
    if (pose.available) {
      metadata_stream << "robot_pose_source: " << pose.source << "\n";
      metadata_stream << "robot_pose_frame_id: " << pose.frame_id << "\n";
      metadata_stream << "robot_pose: [" << pose.x << ", " << pose.y << ", " << pose.yaw << "]\n";
    }

    return true;
  }

  void handle_save_annotated_map(
    const std::shared_ptr<SaveMap::Request> request,
    std::shared_ptr<SaveMap::Response> response)
  {
    response->result = false;
    if (!request->map_topic.empty() && request->map_topic != slam_map_topic_) {
      RCLCPP_ERROR(
        get_logger(),
        "Annotated map saver only supports configured map topic %s (requested %s)",
        slam_map_topic_.c_str(),
        request->map_topic.c_str());
      return;
    }

    auto map = get_latest_slam_map();
    if (!map) {
      RCLCPP_ERROR(get_logger(), "Annotated map saver requested without a cached occupancy grid");
      return;
    }

    AnnotatedMapArtifacts artifacts;
    std::string error;
    if (!resolve_annotated_map_paths(request->map_url, request->image_format, artifacts, error)) {
      RCLCPP_ERROR(get_logger(), "Annotated map save failed: %s", error.c_str());
      return;
    }

    const PoseSnapshot pose = get_best_pose_snapshot(map->header.frame_id);
    if (!render_annotated_map_image(
          map,
          pose,
          artifacts.image_path,
          request->map_mode,
          request->free_thresh,
          request->occupied_thresh,
          artifacts,
          error))
    {
      RCLCPP_ERROR(get_logger(), "Annotated map save failed: %s", error.c_str());
      return;
    }

    if (!write_annotated_map_metadata(
          map,
          pose,
          request->map_mode,
          request->free_thresh,
          request->occupied_thresh,
          artifacts,
          error))
    {
      RCLCPP_ERROR(get_logger(), "Annotated map metadata write failed: %s", error.c_str());
      return;
    }

    response->result = true;
    if (debug_logging_) {
      RCLCPP_INFO(
        get_logger(), "Annotated map saved to %s (metadata: %s)",
        artifacts.image_path.c_str(), artifacts.metadata_path.c_str());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AnnotatedMapSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
