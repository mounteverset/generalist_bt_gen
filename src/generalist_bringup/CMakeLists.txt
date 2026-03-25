cmake_minimum_required(VERSION 3.8)
project(generalist_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(dummy_log_temperature_node
  src/dummy_log_temperature_node.cpp
)
ament_target_dependencies(dummy_log_temperature_node
  rclcpp
  std_srvs
)

add_executable(mock_gps_fix_publisher_node
  src/mock_gps_fix_publisher_node.cpp
)
ament_target_dependencies(mock_gps_fix_publisher_node
  rclcpp
  sensor_msgs
)

install(TARGETS dummy_log_temperature_node
  mock_gps_fix_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
