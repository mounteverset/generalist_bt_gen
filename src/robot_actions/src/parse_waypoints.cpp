#include "robot_actions/parse_waypoints.hpp"

#include <algorithm>
#include <cctype>
#include <deque>
#include <sstream>
#include <string>
#include <vector>

namespace robot_actions
{

namespace
{

void ltrim(std::string & s)
{
  s.erase(
    s.begin(),
    std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
}

void rtrim(std::string & s)
{
  s.erase(
    std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
    s.end());
}

std::vector<std::string> split_waypoints(const std::string & input)
{
  std::vector<std::string> tokens;
  std::string current;
  std::istringstream stream(input);
  while (std::getline(stream, current, ';')) {
    ltrim(current);
    rtrim(current);
    if (!current.empty()) {
      tokens.emplace_back(current);
    }
  }
  return tokens;
}

}  // namespace

ParseWaypoints::ParseWaypoints(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ParseWaypoints::providedPorts()
{
  return {
    BT::InputPort<std::string>("raw_waypoints", "", "Serialized waypoint list."),
    BT::OutputPort<BT::SharedQueue<std::string>>("waypoint_queue", "Parsed waypoint queue.")
  };
}

BT::NodeStatus ParseWaypoints::tick()
{
  const auto raw = getInput<std::string>("raw_waypoints").value_or("");
  auto queue = std::make_shared<std::deque<std::string>>();

  if (!raw.empty()) {
    std::string cleaned = raw;
    ltrim(cleaned);
    rtrim(cleaned);
    if (!cleaned.empty() && cleaned.front() == '[' && cleaned.back() == ']') {
      cleaned = cleaned.substr(1, cleaned.size() - 2);
    }
    const auto tokens = split_waypoints(cleaned);
    for (const auto & token : tokens) {
      queue->push_back(token);
    }
  }

  setOutput("waypoint_queue", queue);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_actions

