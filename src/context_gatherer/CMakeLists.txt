cmake_minimum_required(VERSION 3.8)
project(context_gatherer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(gen_bt_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)

add_executable(context_gatherer_node src/context_gatherer_node.cpp)
add_executable(annotated_map_saver_node src/annotated_map_saver_node.cpp)
add_executable(satellite_map_annotator_node src/satellite_map_annotator_node.cpp)
ament_target_dependencies(context_gatherer_node
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  gen_bt_interfaces
  sensor_msgs
  nav_msgs
  geographic_msgs
  image_transport
  cv_bridge
)
ament_target_dependencies(annotated_map_saver_node
  geometry_msgs
  nav2_msgs
  rclcpp
  nav_msgs
)
ament_target_dependencies(satellite_map_annotator_node
  gen_bt_interfaces
  rclcpp
)

target_link_libraries(context_gatherer_node
  ${OpenCV_LIBS}
  CURL::libcurl
  nlohmann_json::nlohmann_json
)
target_link_libraries(annotated_map_saver_node
  ${OpenCV_LIBS}
)
target_link_libraries(satellite_map_annotator_node
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
)

install(TARGETS context_gatherer_node annotated_map_saver_node satellite_map_annotator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  gen_bt_interfaces
  sensor_msgs
  nav_msgs
  geographic_msgs
  image_transport
  cv_bridge
)

ament_package()
