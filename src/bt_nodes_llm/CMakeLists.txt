cmake_minimum_required(VERSION 3.8)
project(bt_nodes_llm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(llm_interface REQUIRED)

set(bt_nodes_llm_sources
  src/thinking_node.cpp
  src/blackboard_update_node.cpp
  src/plugin_registration.cpp
)

add_library(${PROJECT_NAME} SHARED ${bt_nodes_llm_sources})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp
  rclcpp
  nlohmann_json
  llm_interface)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_bt_nodes_llm
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(bt_nodes_llm_thinking_test
    test/test_thinking_node.cpp)
  if(TARGET bt_nodes_llm_thinking_test)
    target_link_libraries(bt_nodes_llm_thinking_test ${PROJECT_NAME})
    ament_target_dependencies(bt_nodes_llm_thinking_test
      rclcpp
      nlohmann_json
      llm_interface)
    set_tests_properties(bt_nodes_llm_thinking_test PROPERTIES
      ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs;LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$ENV{LD_LIBRARY_PATH}")
  endif()

  ament_add_gtest(bt_nodes_llm_blackboard_update_test
    test/test_blackboard_update_node.cpp)
  if(TARGET bt_nodes_llm_blackboard_update_test)
    target_link_libraries(bt_nodes_llm_blackboard_update_test ${PROJECT_NAME})
    ament_target_dependencies(bt_nodes_llm_blackboard_update_test
      rclcpp
      nlohmann_json
      llm_interface)
    set_tests_properties(bt_nodes_llm_blackboard_update_test PROPERTIES
      ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs;LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$ENV{LD_LIBRARY_PATH}")
  endif()

  if(COMMAND ament_add_launch_test)
    ament_add_launch_test(test/launch/test_llm_bt_nodes_launch.py
      TARGET bt_nodes_llm_launch_test)
    if(TARGET bt_nodes_llm_launch_test)
      set_tests_properties(bt_nodes_llm_launch_test PROPERTIES
        ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs;LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$ENV{LD_LIBRARY_PATH}")
    endif()
  endif()
endif()

ament_export_targets(export_bt_nodes_llm HAS_LIBRARY_TARGET)
ament_export_dependencies(
  behaviortree_cpp
  rclcpp
  nlohmann_json
  llm_interface)

ament_package()
