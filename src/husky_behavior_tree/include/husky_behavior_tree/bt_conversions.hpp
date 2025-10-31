#ifndef HUSKY_BEHAVIOR_TREE_BT_CONVERSIONS_HPP
#define HUSKY_BEHAVIOR_TREE_BT_CONVERSIONS_HPP

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Register custom type converters for BehaviorTree.CPP
namespace BT
{
  /**
   * @brief Convert string to geometry_msgs::msg::PoseStamped
   * 
   * Supported formats:
   * - 7 values: "x;y;z;qx;qy;qz;qw" (uses "map" as default frame_id)
   * - 8 values: "frame_id;x;y;z;qx;qy;qz;qw" (custom frame_id)
   * 
   * Example: "10.0;5.0;0.0;0.0;0.0;0.707;0.707"
   * Example: "map;10.0;5.0;0.0;0.0;0.0;0.707;0.707"
   */
  template <>
  inline geometry_msgs::msg::PoseStamped convertFromString(StringView str)
  {
    // Expected format: "x;y;z;qx;qy;qz;qw" or "frame_id;x;y;z;qx;qy;qz;qw"
    auto parts = splitString(str, ';');
    
    geometry_msgs::msg::PoseStamped pose;
    
    if (parts.size() == 7) {
      // Format: x;y;z;qx;qy;qz;qw
      pose.header.frame_id = "map";  // Default frame
      pose.pose.position.x = convertFromString<double>(parts[0]);
      pose.pose.position.y = convertFromString<double>(parts[1]);
      pose.pose.position.z = convertFromString<double>(parts[2]);
      pose.pose.orientation.x = convertFromString<double>(parts[3]);
      pose.pose.orientation.y = convertFromString<double>(parts[4]);
      pose.pose.orientation.z = convertFromString<double>(parts[5]);
      pose.pose.orientation.w = convertFromString<double>(parts[6]);
    }
    else if (parts.size() == 8) {
      // Format: frame_id;x;y;z;qx;qy;qz;qw
      pose.header.frame_id = std::string(parts[0]);
      pose.pose.position.x = convertFromString<double>(parts[1]);
      pose.pose.position.y = convertFromString<double>(parts[2]);
      pose.pose.position.z = convertFromString<double>(parts[3]);
      pose.pose.orientation.x = convertFromString<double>(parts[4]);
      pose.pose.orientation.y = convertFromString<double>(parts[5]);
      pose.pose.orientation.z = convertFromString<double>(parts[6]);
      pose.pose.orientation.w = convertFromString<double>(parts[7]);
    }
    else {
      throw RuntimeError("Invalid PoseStamped format. Expected 'x;y;z;qx;qy;qz;qw' or 'frame_id;x;y;z;qx;qy;qz;qw'");
    }
    
    return pose;
  }
} // namespace BT

#endif  // HUSKY_BEHAVIOR_TREE_BT_CONVERSIONS_HPP
