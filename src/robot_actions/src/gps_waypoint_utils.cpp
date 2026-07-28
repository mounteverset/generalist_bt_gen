#include "robot_actions/gps_waypoint_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace robot_actions
{

namespace
{

std::string trim_copy(const std::string & input)
{
  const auto front = std::find_if_not(
    input.begin(), input.end(), [](unsigned char c) {return std::isspace(c);});
  const auto back = std::find_if_not(
    input.rbegin(), input.rend(), [](unsigned char c) {return std::isspace(c);}).base();
  if (front >= back) {
    return {};
  }
  return std::string(front, back);
}

}  // namespace

bool parse_gps_waypoint(const std::string & raw, GpsWaypoint & waypoint)
{
  std::string cleaned = raw;
  cleaned.erase(
    std::remove_if(
      cleaned.begin(), cleaned.end(),
      [](unsigned char c) {return c == '[' || c == ']' || c == '(' || c == ')';}),
    cleaned.end());

  std::stringstream stream(cleaned);
  std::string token;
  std::vector<double> values;
  while (std::getline(stream, token, ',')) {
    const auto trimmed = trim_copy(token);
    if (trimmed.empty()) {
      return false;
    }
    try {
      std::size_t consumed = 0;
      const auto value = std::stod(trimmed, &consumed);
      if (consumed != trimmed.size() || !std::isfinite(value)) {
        return false;
      }
      values.push_back(value);
    } catch (const std::exception &) {
      return false;
    }
  }

  if (values.size() < 2 || values.size() > 4) {
    return false;
  }
  if (values[0] < -90.0 || values[0] > 90.0 ||
    values[1] < -180.0 || values[1] > 180.0)
  {
    return false;
  }

  waypoint.latitude = values[0];
  waypoint.longitude = values[1];
  waypoint.altitude = values.size() == 4 ? values[2] : 0.0;
  waypoint.yaw = values.size() >= 3 ? values.back() : 0.0;
  return true;
}

std::vector<std::string> split_gps_waypoints(const std::string & raw)
{
  std::string cleaned = trim_copy(raw);
  if (cleaned.size() >= 2 && cleaned.front() == '[' && cleaned.back() == ']') {
    cleaned = cleaned.substr(1, cleaned.size() - 2);
  }

  std::vector<std::string> waypoints;
  std::stringstream stream(cleaned);
  std::string token;
  while (std::getline(stream, token, ';')) {
    const auto trimmed = trim_copy(token);
    if (!trimmed.empty()) {
      waypoints.push_back(trimmed);
    }
  }
  return waypoints;
}

}  // namespace robot_actions
