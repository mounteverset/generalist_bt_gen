#include "robot_actions/parse_gps_waypoints.hpp"

#include "robot_actions/gps_waypoint_utils.hpp"

#include <deque>
#include <memory>
#include <string>

namespace robot_actions
{

ParseGpsWaypoints::ParseGpsWaypoints(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ParseGpsWaypoints::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "raw_waypoints", "",
      "Semicolon-separated GPS waypoints formatted as lat,lon[,yaw] or lat,lon,alt,yaw."),
    BT::OutputPort<BT::SharedQueue<std::string>>(
      "waypoint_queue", "Validated GPS waypoint queue."),
    BT::OutputPort<int>("waypoint_count", "Number of parsed GPS waypoints.")
  };
}

BT::NodeStatus ParseGpsWaypoints::tick()
{
  const auto raw = getInput<std::string>("raw_waypoints").value_or("");
  auto queue = std::make_shared<std::deque<std::string>>();

  for (const auto & token : split_gps_waypoints(raw)) {
    GpsWaypoint waypoint;
    if (!parse_gps_waypoint(token, waypoint)) {
      return BT::NodeStatus::FAILURE;
    }
    queue->push_back(token);
  }

  if (queue->empty()) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("waypoint_count", static_cast<int>(queue->size()));
  setOutput("waypoint_queue", queue);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_actions
