#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <language_feature_msgs/srv/find_object_locations.hpp>

#include <string>

namespace robot_actions
{

class FindObjectLocation
: public BT::RosServiceNode<language_feature_msgs::srv::FindObjectLocations>
{
public:
  FindObjectLocation(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr & request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  bool enable_debug_logging_{false};
  std::string query_;
  double yaw_{0.0};
};

}  // namespace robot_actions
