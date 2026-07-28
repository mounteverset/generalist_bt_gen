#pragma once

#include <string>
#include <vector>

namespace robot_actions
{

struct GpsWaypoint
{
  double latitude{0.0};
  double longitude{0.0};
  double altitude{0.0};
  double yaw{0.0};
};

bool parse_gps_waypoint(const std::string & raw, GpsWaypoint & waypoint);

std::vector<std::string> split_gps_waypoints(const std::string & raw);

}  // namespace robot_actions
