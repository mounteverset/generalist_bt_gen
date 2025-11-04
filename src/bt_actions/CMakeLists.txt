cmake_minimum_required(VERSION 3.8)
project(bt_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(bt_actions_sources
  src/navigate_action.cpp
  src/take_photo_action.cpp
  src/detect_tree_condition.cpp
  src/explore_field_action.cpp
  src/plugin_registration.cpp
)

add_library(${PROJECT_NAME} SHARED ${bt_actions_sources})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  nav2_msgs
  sensor_msgs
  action_msgs)


install(TARGETS ${PROJECT_NAME}
  EXPORT export_bt_actions
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(bt_actions_plugins_test
    test/test_bt_actions_plugins.cpp)
  if(TARGET bt_actions_plugins_test)
    target_link_libraries(bt_actions_plugins_test ${PROJECT_NAME})
    ament_target_dependencies(bt_actions_plugins_test
      rclcpp
      rclcpp_action
      behaviortree_cpp
      geometry_msgs
      nav2_msgs
      sensor_msgs
      action_msgs)
    set_tests_properties(bt_actions_plugins_test PROPERTIES
      ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs;LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$ENV{LD_LIBRARY_PATH}")
  endif()
  if(COMMAND ament_add_launch_test)
    ament_add_launch_test(test/launch/test_bt_actions_launch.py
      TARGET bt_actions_launch_test)
    if(TARGET bt_actions_launch_test)
      set_tests_properties(bt_actions_launch_test PROPERTIES
        ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs;LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$ENV{LD_LIBRARY_PATH}")
    endif()
  endif()
endif()

ament_export_targets(export_bt_actions HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  nav2_msgs
  sensor_msgs
  action_msgs)

ament_package()
