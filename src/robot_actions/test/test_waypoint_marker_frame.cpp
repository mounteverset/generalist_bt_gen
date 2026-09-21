#include "robot_actions/publish_waypoint_markers.hpp"

#include <gtest/gtest.h>

TEST(WaypointMarkerFrame, FallsBackOnlyWhenRequestedFrameIsUnavailable)
{
  const std::vector<std::string> available_frames{"map", "target/map"};

  EXPECT_EQ(robot_actions::marker_frame_or_map("target/map", available_frames), "target/map");
  EXPECT_EQ(robot_actions::marker_frame_or_map("target/odom", available_frames), "map");
  EXPECT_EQ(robot_actions::marker_frame_or_map("map", {}), "map");
}
