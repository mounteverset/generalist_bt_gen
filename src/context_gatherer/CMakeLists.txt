cmake_minimum_required(VERSION 3.8)
project(context_gatherer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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

add_executable(context_gatherer_node src/context_gatherer_node.cpp)
ament_target_dependencies(context_gatherer_node
  rclcpp
  rclcpp_action
  gen_bt_interfaces
  sensor_msgs
  nav_msgs
  geographic_msgs
  image_transport
  cv_bridge
)

target_link_libraries(context_gatherer_node
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
)

install(TARGETS context_gatherer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
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
