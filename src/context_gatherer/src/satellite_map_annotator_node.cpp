#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "gen_bt_interfaces/srv/annotate_satellite_map.hpp"
#include "nlohmann/json.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;

class SatelliteMapAnnotatorNode : public rclcpp::Node
{
public:
  using AnnotateSatelliteMap = gen_bt_interfaces::srv::AnnotateSatelliteMap;

  SatelliteMapAnnotatorNode()
  : Node("satellite_map_annotator")
  {
    output_directory_ = this->declare_parameter<std::string>("output_directory", "/tmp/context_gatherer");
    service_name_ = this->declare_parameter<std::string>(
      "satellite_map_annotator_service_name", "/satellite_map_annotator/annotate");
    enable_debug_logging_ = this->declare_parameter<bool>("enable_debug_logging", true);
    default_draw_lat_lon_grid_ = this->declare_parameter<bool>("draw_lat_lon_grid", true);
    default_draw_scale_bar_ = this->declare_parameter<bool>("draw_scale_bar", true);
    default_draw_labels_ = this->declare_parameter<bool>("draw_labels", true);

    std::filesystem::create_directories(output_directory_);

    service_ = this->create_service<AnnotateSatelliteMap>(
      service_name_,
      std::bind(
        &SatelliteMapAnnotatorNode::handle_annotate_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Satellite map annotator ready on %s (output_dir=%s)",
      service_name_.c_str(),
      output_directory_.c_str());
  }

private:
  struct GeoPoint
  {
    double lat{0.0};
    double lon{0.0};
    std::string label;
  };

  struct GeoBounds
  {
    double north{0.0};
    double south{0.0};
    double east{0.0};
    double west{0.0};

    bool valid() const
    {
      return std::isfinite(north) && std::isfinite(south) && std::isfinite(east) &&
             std::isfinite(west) && north > south && east > west;
    }
  };

  struct PixelPoint
  {
    int x{0};
    int y{0};
  };

  rclcpp::Service<AnnotateSatelliteMap>::SharedPtr service_;

  std::string output_directory_;
  std::string service_name_;
  bool enable_debug_logging_;
  bool default_draw_lat_lon_grid_;
  bool default_draw_scale_bar_;
  bool default_draw_labels_;

  void handle_annotate_request(
    const std::shared_ptr<AnnotateSatelliteMap::Request> request,
    std::shared_ptr<AnnotateSatelliteMap::Response> response)
  {
    json request_json;
    try {
      request_json = request->request_json.empty() ? json::object() : json::parse(request->request_json);
      if (!request_json.is_object()) {
        response->success = false;
        response->message = "request_json must be a JSON object";
        return;
      }
    } catch (const std::exception& exc) {
      response->success = false;
      response->message = std::string("Failed to parse request_json: ") + exc.what();
      return;
    }

    std::string source_image_uri = request->source_image_uri;
    if (source_image_uri.empty()) {
      source_image_uri = extract_first_string(
        request_json,
        {"source_image_uri", "image_uri", "uri", "source_image_path", "image_path"});
      if (!source_image_uri.empty() && source_image_uri.rfind("file://", 0) != 0 &&
          source_image_uri.front() != '/')
      {
        source_image_uri.clear();
      }
    }

    if (source_image_uri.empty()) {
      response->success = false;
      response->message = "Satellite map annotation requires source_image_uri or image_path";
      return;
    }

    const std::string source_image_path = strip_file_uri(source_image_uri);
    cv::Mat image = cv::imread(source_image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
      response->success = false;
      response->message = "Failed to load satellite map image from " + source_image_path;
      return;
    }

    const GeoBounds bounds = extract_bounds(request_json);
    const bool draw_grid = request_json.value("draw_lat_lon_grid", default_draw_lat_lon_grid_);
    const bool draw_scale_bar = request_json.value("draw_scale_bar", default_draw_scale_bar_);
    const bool draw_labels = request_json.value("draw_labels", default_draw_labels_);

    std::vector<std::string> overlays_applied;

    if (bounds.valid() && draw_grid) {
      draw_lat_lon_grid(image, bounds);
      overlays_applied.push_back("lat_lon_grid");
    }

    const auto mission_boundary = extract_polygon(
      request_json, {"mission_boundary", "field_boundary", "boundary"});
    if (!mission_boundary.empty() && bounds.valid()) {
      draw_polygon(image, mission_boundary, bounds, cv::Scalar(14, 165, 233), 3, draw_labels, "Boundary");
      overlays_applied.push_back("mission_boundary");
    }

    const auto highlighted_places = extract_points(
      request_json, {"points_of_interest", "highlighted_places", "markers"});
    if (!highlighted_places.empty() && bounds.valid()) {
      draw_markers(image, highlighted_places, bounds, cv::Scalar(34, 197, 94), draw_labels);
      overlays_applied.push_back("points_of_interest");
    }

    const auto robot_location = extract_single_point(request_json, {"robot_location", "robot_pose"});
    if (robot_location.has_value() && bounds.valid()) {
      draw_robot_marker(image, robot_location.value(), bounds, request_json);
      overlays_applied.push_back("robot_location");
    }

    const std::optional<double> meters_per_px = bounds.valid()
      ? estimate_meters_per_px(bounds, image.cols)
      : std::nullopt;
    if (meters_per_px.has_value() && draw_scale_bar) {
      draw_scale_bar_overlay(image, meters_per_px.value());
      overlays_applied.push_back("scale_bar");
    }

    if (draw_labels) {
      draw_header(image, request_json, bounds, meters_per_px);
      overlays_applied.push_back("header");
    }

    const std::string safe_session_id = sanitize_for_filename(request->session_id);
    const std::string basename = safe_session_id.empty()
      ? "satellite_map"
      : "satellite_map_" + safe_session_id;
    const std::string image_path = build_timestamped_path(basename, "png");
    const std::string metadata_path = build_timestamped_path(basename, "json");

    if (!cv::imwrite(image_path, image)) {
      response->success = false;
      response->message = "Failed to write annotated satellite map to disk";
      return;
    }

    json metadata = build_metadata_json(
      request_json,
      source_image_uri,
      image,
      bounds,
      meters_per_px,
      overlays_applied);

    std::ofstream metadata_stream(metadata_path);
    if (!metadata_stream) {
      response->success = false;
      response->message = "Failed to write satellite metadata file";
      return;
    }
    metadata_stream << metadata.dump(2);

    response->success = true;
    response->annotated_image_uri = to_file_uri(image_path);
    response->metadata_uri = to_file_uri(metadata_path);
    response->metadata_json = metadata.dump();
    response->message = "Satellite map annotated successfully";

    if (enable_debug_logging_) {
      RCLCPP_INFO(
        get_logger(),
        "Annotated satellite map -> %s (overlays=%zu)",
        response->annotated_image_uri.c_str(),
        overlays_applied.size());
    }
  }

  json build_metadata_json(
    const json& request_json,
    const std::string& source_image_uri,
    const cv::Mat& image,
    const GeoBounds& bounds,
    const std::optional<double>& meters_per_px,
    const std::vector<std::string>& overlays_applied) const
  {
    json metadata = {
      {"source_image_uri", source_image_uri},
      {"source", request_json.value("source", std::string("unknown"))},
      {"zoom", request_json.contains("zoom") ? request_json["zoom"] : json(nullptr)},
      {"image_width_px", image.cols},
      {"image_height_px", image.rows},
      {"draw_lat_lon_grid", request_json.value("draw_lat_lon_grid", default_draw_lat_lon_grid_)},
      {"draw_scale_bar", request_json.value("draw_scale_bar", default_draw_scale_bar_)},
      {"overlays_applied", overlays_applied}
    };

    if (bounds.valid()) {
      metadata["bounds"] = {
        {"north", bounds.north},
        {"south", bounds.south},
        {"east", bounds.east},
        {"west", bounds.west}
      };
      metadata["center"] = {
        {"lat", (bounds.north + bounds.south) / 2.0},
        {"lon", (bounds.east + bounds.west) / 2.0}
      };
    }

    if (meters_per_px.has_value()) {
      metadata["meters_per_px"] = meters_per_px.value();
    }

    const auto boundary = extract_polygon(request_json, {"mission_boundary", "field_boundary", "boundary"});
    if (!boundary.empty()) {
      metadata["mission_boundary"] = polygon_to_json(boundary);
    }

    const auto points = extract_points(request_json, {"points_of_interest", "highlighted_places", "markers"});
    if (!points.empty()) {
      metadata["points_of_interest"] = points_to_json(points);
    }

    const auto robot_location = extract_single_point(request_json, {"robot_location", "robot_pose"});
    if (robot_location.has_value()) {
      metadata["robot_location"] = {
        {"lat", robot_location->lat},
        {"lon", robot_location->lon},
        {"label", robot_location->label}
      };
    }

    return metadata;
  }

  void draw_header(
    cv::Mat& image,
    const json& request_json,
    const GeoBounds& bounds,
    const std::optional<double>& meters_per_px) const
  {
    const int panel_height = std::max(48, image.rows / 12);
    cv::rectangle(
      image,
      cv::Rect(0, 0, image.cols, panel_height),
      cv::Scalar(15, 23, 42),
      cv::FILLED);
    cv::rectangle(
      image,
      cv::Rect(0, 0, image.cols, panel_height),
      cv::Scalar(56, 189, 248),
      2);

    std::ostringstream title;
    title << "Satellite Context";
    const std::string source = request_json.value("source", std::string());
    if (!source.empty()) {
      title << " | " << source;
    }
    if (request_json.contains("zoom")) {
      title << " | z=" << request_json["zoom"];
    }
    cv::putText(
      image,
      title.str(),
      cv::Point(16, panel_height / 2),
      cv::FONT_HERSHEY_SIMPLEX,
      0.65,
      cv::Scalar(248, 250, 252),
      2,
      cv::LINE_AA);

    if (bounds.valid()) {
      std::ostringstream subtitle;
      subtitle << std::fixed << std::setprecision(5)
               << "N " << bounds.north << "  S " << bounds.south
               << "  E " << bounds.east << "  W " << bounds.west;
      if (meters_per_px.has_value()) {
        subtitle << "  |  " << std::setprecision(2) << meters_per_px.value() << " m/px";
      }
      cv::putText(
        image,
        subtitle.str(),
        cv::Point(16, panel_height - 14),
        cv::FONT_HERSHEY_SIMPLEX,
        0.45,
        cv::Scalar(203, 213, 225),
        1,
        cv::LINE_AA);
    }
  }

  void draw_lat_lon_grid(cv::Mat& image, const GeoBounds& bounds) const
  {
    const double lat_step = choose_grid_step(bounds.north - bounds.south);
    const double lon_step = choose_grid_step(bounds.east - bounds.west);
    const cv::Scalar grid_color(71, 85, 105);
    const cv::Scalar label_bg(15, 23, 42);

    const double first_lat = std::ceil(bounds.south / lat_step) * lat_step;
    for (double lat = first_lat; lat < bounds.north; lat += lat_step) {
      const auto px = lon_lat_to_pixel(bounds.west, lat, bounds, image.cols, image.rows);
      cv::line(image, cv::Point(0, px.y), cv::Point(image.cols - 1, px.y), grid_color, 1, cv::LINE_AA);
      draw_small_label(image, format_lat_lon(lat, true), cv::Point(8, std::max(18, px.y - 4)), label_bg);
    }

    const double first_lon = std::ceil(bounds.west / lon_step) * lon_step;
    for (double lon = first_lon; lon < bounds.east; lon += lon_step) {
      const auto px = lon_lat_to_pixel(lon, bounds.south, bounds, image.cols, image.rows);
      cv::line(image, cv::Point(px.x, 0), cv::Point(px.x, image.rows - 1), grid_color, 1, cv::LINE_AA);
      draw_small_label(
        image,
        format_lat_lon(lon, false),
        cv::Point(std::min(image.cols - 80, px.x + 4), image.rows - 12),
        label_bg);
    }
  }

  void draw_polygon(
    cv::Mat& image,
    const std::vector<GeoPoint>& polygon,
    const GeoBounds& bounds,
    const cv::Scalar& color,
    int thickness,
    bool draw_labels,
    const std::string& label) const
  {
    if (polygon.size() < 2) {
      return;
    }

    std::vector<cv::Point> pixels;
    pixels.reserve(polygon.size());
    for (const auto& point : polygon) {
      const auto px = lon_lat_to_pixel(point.lon, point.lat, bounds, image.cols, image.rows);
      pixels.push_back(cv::Point(px.x, px.y));
    }

    const cv::Point* pts = pixels.data();
    const int count = static_cast<int>(pixels.size());
    cv::polylines(image, &pts, &count, 1, true, color, thickness, cv::LINE_AA);

    if (draw_labels && !label.empty()) {
      draw_small_label(image, label, pixels.front() + cv::Point(8, -8), cv::Scalar(15, 23, 42));
    }
  }

  void draw_markers(
    cv::Mat& image,
    const std::vector<GeoPoint>& points,
    const GeoBounds& bounds,
    const cv::Scalar& color,
    bool draw_labels) const
  {
    for (const auto& point : points) {
      const auto px = lon_lat_to_pixel(point.lon, point.lat, bounds, image.cols, image.rows);
      cv::circle(image, cv::Point(px.x, px.y), 7, color, cv::FILLED, cv::LINE_AA);
      cv::circle(image, cv::Point(px.x, px.y), 9, cv::Scalar(15, 23, 42), 2, cv::LINE_AA);
      if (draw_labels && !point.label.empty()) {
        draw_small_label(image, point.label, cv::Point(px.x + 10, px.y - 10), cv::Scalar(15, 23, 42));
      }
    }
  }

  void draw_robot_marker(
    cv::Mat& image,
    const GeoPoint& robot_location,
    const GeoBounds& bounds,
    const json& request_json) const
  {
    const auto px = lon_lat_to_pixel(
      robot_location.lon, robot_location.lat, bounds, image.cols, image.rows);
    const double heading_deg = request_json.value("robot_heading_deg", request_json.value("heading_deg", 0.0));
    const double heading_rad = heading_deg * pi() / 180.0;
    const int marker_size = std::max(10, std::min(image.cols, image.rows) / 40);

    cv::Point tip(
      static_cast<int>(std::round(px.x + std::cos(heading_rad) * marker_size)),
      static_cast<int>(std::round(px.y - std::sin(heading_rad) * marker_size)));
    cv::arrowedLine(
      image,
      cv::Point(px.x, px.y),
      tip,
      cv::Scalar(239, 68, 68),
      3,
      cv::LINE_AA,
      0,
      0.3);
    cv::circle(image, cv::Point(px.x, px.y), 7, cv::Scalar(239, 68, 68), cv::FILLED, cv::LINE_AA);
    draw_small_label(
      image,
      robot_location.label.empty() ? "Robot" : robot_location.label,
      cv::Point(px.x + 10, px.y + 18),
      cv::Scalar(15, 23, 42));
  }

  void draw_scale_bar_overlay(cv::Mat& image, double meters_per_px) const
  {
    if (!std::isfinite(meters_per_px) || meters_per_px <= 0.0) {
      return;
    }

    const int max_px = std::max(60, image.cols / 5);
    const double max_meters = meters_per_px * static_cast<double>(max_px);
    const double bar_meters = choose_scale_bar_length(max_meters);
    const int bar_px = std::max(40, static_cast<int>(std::round(bar_meters / meters_per_px)));
    const int x = 24;
    const int y = image.rows - 28;

    cv::line(image, cv::Point(x, y), cv::Point(x + bar_px, y), cv::Scalar(248, 250, 252), 3, cv::LINE_AA);
    cv::line(image, cv::Point(x, y - 6), cv::Point(x, y + 6), cv::Scalar(248, 250, 252), 2, cv::LINE_AA);
    cv::line(
      image, cv::Point(x + bar_px, y - 6), cv::Point(x + bar_px, y + 6), cv::Scalar(248, 250, 252), 2,
      cv::LINE_AA);

    std::ostringstream label;
    if (bar_meters >= 1000.0) {
      label << std::fixed << std::setprecision(1) << (bar_meters / 1000.0) << " km";
    } else {
      label << std::fixed << std::setprecision(0) << bar_meters << " m";
    }
    draw_small_label(image, label.str(), cv::Point(x, y - 12), cv::Scalar(15, 23, 42));
  }

  static double choose_scale_bar_length(double max_meters)
  {
    const double magnitude = std::pow(10.0, std::floor(std::log10(std::max(1.0, max_meters))));
    for (double factor : {1.0, 2.0, 5.0}) {
      const double candidate = factor * magnitude;
      if (candidate >= max_meters * 0.45 && candidate <= max_meters) {
        return candidate;
      }
    }
    return magnitude;
  }

  static std::string format_lat_lon(double value, bool latitude)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(5) << std::abs(value)
           << (latitude ? (value >= 0.0 ? "N" : "S") : (value >= 0.0 ? "E" : "W"));
    return stream.str();
  }

  void draw_small_label(
    cv::Mat& image,
    const std::string& text,
    const cv::Point& anchor,
    const cv::Scalar& background) const
  {
    if (text.empty()) {
      return;
    }

    int baseline = 0;
    const double font_scale = 0.42;
    const int thickness = 1;
    const cv::Size text_size = cv::getTextSize(
      text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
    const int padding = 4;
    const int x = std::clamp(anchor.x, 0, std::max(0, image.cols - text_size.width - padding * 2));
    const int y = std::clamp(anchor.y, text_size.height + padding, std::max(text_size.height + padding, image.rows - padding));

    cv::rectangle(
      image,
      cv::Rect(
        x - padding,
        y - text_size.height - padding,
        text_size.width + padding * 2,
        text_size.height + padding * 2),
      background,
      cv::FILLED);
    cv::putText(
      image,
      text,
      cv::Point(x, y),
      cv::FONT_HERSHEY_SIMPLEX,
      font_scale,
      cv::Scalar(248, 250, 252),
      thickness,
      cv::LINE_AA);
  }

  static PixelPoint lon_lat_to_pixel(
    double lon,
    double lat,
    const GeoBounds& bounds,
    int image_width,
    int image_height)
  {
    const double x_norm = (lon - bounds.west) / (bounds.east - bounds.west);
    const double y_norm = (bounds.north - lat) / (bounds.north - bounds.south);
    return {
      static_cast<int>(std::round(std::clamp(x_norm, 0.0, 1.0) * static_cast<double>(image_width - 1))),
      static_cast<int>(std::round(std::clamp(y_norm, 0.0, 1.0) * static_cast<double>(image_height - 1)))
    };
  }

  static std::optional<double> estimate_meters_per_px(const GeoBounds& bounds, int image_width)
  {
    if (!bounds.valid() || image_width <= 0) {
      return std::nullopt;
    }

    const double center_lat = (bounds.north + bounds.south) / 2.0;
    const double meters = haversine_distance_m(center_lat, bounds.west, center_lat, bounds.east);
    if (!std::isfinite(meters) || meters <= 0.0) {
      return std::nullopt;
    }
    return meters / static_cast<double>(image_width);
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

  static double choose_grid_step(double span_deg)
  {
    const double target = std::max(span_deg / 6.0, 1e-5);
    const double magnitude = std::pow(10.0, std::floor(std::log10(target)));
    for (double factor : {1.0, 2.0, 5.0, 10.0}) {
      const double candidate = factor * magnitude;
      if (candidate >= target) {
        return candidate;
      }
    }
    return magnitude * 10.0;
  }

  static std::string to_file_uri(const std::string& path)
  {
    return "file://" + std::filesystem::absolute(path).string();
  }

  static constexpr double pi()
  {
    return 3.14159265358979323846;
  }

  static std::string strip_file_uri(const std::string& uri)
  {
    constexpr const char* prefix = "file://";
    if (uri.rfind(prefix, 0) == 0) {
      return uri.substr(7);
    }
    return uri;
  }

  std::string build_timestamped_path(const std::string& prefix, const std::string& extension) const
  {
    const auto stamp_ns = this->now().nanoseconds();
    return output_directory_ + "/" + prefix + "_" + std::to_string(stamp_ns) + "." + extension;
  }

  static std::string sanitize_for_filename(const std::string& raw)
  {
    std::string sanitized;
    sanitized.reserve(raw.size());
    for (char c : raw) {
      if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
        sanitized.push_back(c);
      } else if (!sanitized.empty() && sanitized.back() != '_') {
        sanitized.push_back('_');
      }
    }
    return sanitized;
  }

  static std::string extract_first_string(const json& input, const std::vector<std::string>& keys)
  {
    if (!input.is_object()) {
      return "";
    }
    for (const auto& key : keys) {
      const auto it = input.find(key);
      if (it != input.end() && it->is_string()) {
        return it->get<std::string>();
      }
    }
    return "";
  }

  static std::optional<double> extract_number_from_object(const json& object, const std::string& key)
  {
    if (!object.is_object()) {
      return std::nullopt;
    }
    const auto it = object.find(key);
    if (it == object.end()) {
      return std::nullopt;
    }
    if (it->is_number()) {
      return it->get<double>();
    }
    if (it->is_string()) {
      try {
        return std::stod(it->get<std::string>());
      } catch (...) {
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  static GeoBounds extract_bounds(const json& request_json)
  {
    if (!request_json.is_object()) {
      return {};
    }

    json bounds_json;
    if (request_json.contains("bounds") && request_json["bounds"].is_object()) {
      bounds_json = request_json["bounds"];
    } else if (request_json.contains("bbox") && request_json["bbox"].is_object()) {
      bounds_json = request_json["bbox"];
    }

    if (!bounds_json.is_object()) {
      return {};
    }

    GeoBounds bounds;
    bounds.north = extract_number_from_object(bounds_json, "north").value_or(
      extract_number_from_object(bounds_json, "max_lat").value_or(0.0));
    bounds.south = extract_number_from_object(bounds_json, "south").value_or(
      extract_number_from_object(bounds_json, "min_lat").value_or(0.0));
    bounds.east = extract_number_from_object(bounds_json, "east").value_or(
      extract_number_from_object(bounds_json, "max_lon").value_or(0.0));
    bounds.west = extract_number_from_object(bounds_json, "west").value_or(
      extract_number_from_object(bounds_json, "min_lon").value_or(0.0));
    return bounds;
  }

  static std::optional<GeoPoint> extract_single_point(
    const json& request_json,
    const std::vector<std::string>& keys)
  {
    if (!request_json.is_object()) {
      return std::nullopt;
    }

    for (const auto& key : keys) {
      const auto it = request_json.find(key);
      if (it == request_json.end()) {
        continue;
      }
      const auto point = parse_point(*it);
      if (point.has_value()) {
        return point;
      }
    }
    return std::nullopt;
  }

  static std::vector<GeoPoint> extract_points(
    const json& request_json,
    const std::vector<std::string>& keys)
  {
    if (!request_json.is_object()) {
      return {};
    }

    for (const auto& key : keys) {
      const auto it = request_json.find(key);
      if (it == request_json.end()) {
        continue;
      }
      if (it->is_array()) {
        std::vector<GeoPoint> points;
        for (const auto& item : *it) {
          const auto point = parse_point(item);
          if (point.has_value()) {
            points.push_back(point.value());
          }
        }
        if (!points.empty()) {
          return points;
        }
      } else {
        const auto point = parse_point(*it);
        if (point.has_value()) {
          return {point.value()};
        }
      }
    }
    return {};
  }

  static std::vector<GeoPoint> extract_polygon(
    const json& request_json,
    const std::vector<std::string>& keys)
  {
    if (!request_json.is_object()) {
      return {};
    }

    for (const auto& key : keys) {
      const auto it = request_json.find(key);
      if (it == request_json.end()) {
        continue;
      }
      const auto polygon = parse_polygon(*it);
      if (!polygon.empty()) {
        return polygon;
      }
    }
    return {};
  }

  static std::optional<GeoPoint> parse_point(const json& point_json)
  {
    if (!point_json.is_object()) {
      return std::nullopt;
    }

    const auto lat = extract_number_from_object(point_json, "lat").value_or(
      extract_number_from_object(point_json, "latitude").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto lon = extract_number_from_object(point_json, "lon").value_or(
      extract_number_from_object(point_json, "lng").value_or(
        extract_number_from_object(point_json, "longitude").value_or(std::numeric_limits<double>::quiet_NaN())));
    if (!std::isfinite(lat) || !std::isfinite(lon)) {
      return std::nullopt;
    }

    GeoPoint point;
    point.lat = lat;
    point.lon = lon;
    if (point_json.contains("label") && point_json["label"].is_string()) {
      point.label = point_json["label"].get<std::string>();
    } else if (point_json.contains("name") && point_json["name"].is_string()) {
      point.label = point_json["name"].get<std::string>();
    }
    return point;
  }

  static std::vector<GeoPoint> parse_polygon(const json& polygon_json)
  {
    if (polygon_json.is_array()) {
      std::vector<GeoPoint> points;
      for (const auto& item : polygon_json) {
        const auto point = parse_point(item);
        if (point.has_value()) {
          points.push_back(point.value());
        }
      }
      return points;
    }

    if (!polygon_json.is_object()) {
      return {};
    }

    if (polygon_json.contains("points")) {
      return parse_polygon(polygon_json["points"]);
    }

    if (polygon_json.contains("coordinates") && polygon_json["coordinates"].is_array()) {
      const auto& coordinates = polygon_json["coordinates"];
      if (!coordinates.empty() && coordinates.front().is_array()) {
        return parse_coordinate_ring(coordinates.front());
      }
      return parse_coordinate_ring(coordinates);
    }

    return {};
  }

  static std::vector<GeoPoint> parse_coordinate_ring(const json& ring_json)
  {
    std::vector<GeoPoint> points;
    if (!ring_json.is_array()) {
      return points;
    }

    for (const auto& item : ring_json) {
      if (!item.is_array() || item.size() < 2) {
        continue;
      }
      if (!item[0].is_number() || !item[1].is_number()) {
        continue;
      }
      GeoPoint point;
      point.lon = item[0].get<double>();
      point.lat = item[1].get<double>();
      points.push_back(point);
    }
    return points;
  }

  static json polygon_to_json(const std::vector<GeoPoint>& polygon)
  {
    json points = json::array();
    for (const auto& point : polygon) {
      points.push_back({
        {"lat", point.lat},
        {"lon", point.lon},
        {"label", point.label}
      });
    }
    return points;
  }

  static json points_to_json(const std::vector<GeoPoint>& points)
  {
    json array = json::array();
    for (const auto& point : points) {
      array.push_back({
        {"lat", point.lat},
        {"lon", point.lon},
        {"label", point.label}
      });
    }
    return array;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SatelliteMapAnnotatorNode>());
  rclcpp::shutdown();
  return 0;
}
