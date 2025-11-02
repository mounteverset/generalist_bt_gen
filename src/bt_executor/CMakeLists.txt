cmake_minimum_required(VERSION 3.8)
project(bt_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

set(bt_executor_sources
  src/bt_executor.cpp
  src/bt_loader.cpp
  src/bt_failure_handler.cpp
  src/bt_monitor.cpp
)

add_library(bt_executor SHARED ${bt_executor_sources})
target_include_directories(bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bt_executor yaml-cpp)
ament_target_dependencies(bt_executor
  rclcpp
  behaviortree_cpp
  yaml-cpp
  nlohmann_json
  std_srvs
  std_msgs)

add_executable(bt_executor_node src/bt_executor_node.cpp)
target_link_libraries(bt_executor_node bt_executor)
ament_target_dependencies(bt_executor_node rclcpp)

install(TARGETS
  bt_executor_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS bt_executor
  EXPORT export_bt_executor
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY
  config
  launch
  trees
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_bt_executor HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  yaml-cpp
  nlohmann_json
  std_srvs
  std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(bt_executor_components_test
    test/test_bt_components.cpp)
  if(TARGET bt_executor_components_test)
    target_link_libraries(bt_executor_components_test bt_executor)
    set_tests_properties(bt_executor_components_test PROPERTIES
      ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs")
  endif()
  if(COMMAND ament_add_launch_test)
    ament_add_launch_test(test/launch/test_bt_executor_launch.py
      TARGET bt_executor_launch_test)
    if(TARGET bt_executor_launch_test)
      set_tests_properties(bt_executor_launch_test PROPERTIES
        ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs")
    endif()
  endif()
endif()

ament_package()
