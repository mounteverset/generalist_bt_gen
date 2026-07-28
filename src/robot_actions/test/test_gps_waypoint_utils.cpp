#include "robot_actions/gps_waypoint_utils.hpp"

#include <gtest/gtest.h>

TEST(GpsWaypointUtils, ParsesLatitudeLongitudeAndYaw)
{
  robot_actions::GpsWaypoint waypoint;

  ASSERT_TRUE(robot_actions::parse_gps_waypoint("48.2848,11.6077,2.27", waypoint));
  EXPECT_DOUBLE_EQ(waypoint.latitude, 48.2848);
  EXPECT_DOUBLE_EQ(waypoint.longitude, 11.6077);
  EXPECT_DOUBLE_EQ(waypoint.altitude, 0.0);
  EXPECT_DOUBLE_EQ(waypoint.yaw, 2.27);
}

TEST(GpsWaypointUtils, ParsesAltitudeAndYaw)
{
  robot_actions::GpsWaypoint waypoint;

  ASSERT_TRUE(robot_actions::parse_gps_waypoint("48.2848,11.6077,473.0,-1.2", waypoint));
  EXPECT_DOUBLE_EQ(waypoint.altitude, 473.0);
  EXPECT_DOUBLE_EQ(waypoint.yaw, -1.2);
}

TEST(GpsWaypointUtils, RejectsInvalidCoordinates)
{
  robot_actions::GpsWaypoint waypoint;

  EXPECT_FALSE(robot_actions::parse_gps_waypoint("91.0,11.0,0.0", waypoint));
  EXPECT_FALSE(robot_actions::parse_gps_waypoint("48.0,181.0,0.0", waypoint));
  EXPECT_FALSE(robot_actions::parse_gps_waypoint("48.0,11.0,0.0,0.0,1.0", waypoint));
  EXPECT_FALSE(robot_actions::parse_gps_waypoint("not-a-latitude,11.0", waypoint));
  EXPECT_FALSE(robot_actions::parse_gps_waypoint("48.0,11.0degrees", waypoint));
}

TEST(GpsWaypointUtils, SplitsSemicolonSeparatedWaypoints)
{
  const auto waypoints = robot_actions::split_gps_waypoints(
    "48.1,11.1,0.0; 48.2,11.2,1.57");

  ASSERT_EQ(waypoints.size(), 2U);
  EXPECT_EQ(waypoints[0], "48.1,11.1,0.0");
  EXPECT_EQ(waypoints[1], "48.2,11.2,1.57");
}
