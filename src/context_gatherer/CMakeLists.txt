cmake_minimum_required(VERSION 3.8)
project(context_gatherer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

set(context_gatherer_sources
  src/context_gatherer_node.cpp
)

add_library(${PROJECT_NAME} SHARED ${context_gatherer_sources})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  nav_msgs
  geographic_msgs
  cv_bridge
  image_transport
  nlohmann_json
  std_srvs)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS})

add_executable(context_gatherer_node src/context_gatherer_main.cpp)
target_link_libraries(context_gatherer_node ${PROJECT_NAME})
ament_target_dependencies(context_gatherer_node
  rclcpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_context_gatherer
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS context_gatherer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_library(context_gatherer_mock_sensor STATIC
    test/mock_sensor_publisher.cpp)
  target_include_directories(context_gatherer_mock_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
  ament_target_dependencies(context_gatherer_mock_sensor
    rclcpp
    sensor_msgs
    nav_msgs
    geographic_msgs
    image_transport)

  ament_add_gtest(context_gatherer_mock_sensor_test
    test/test_mock_sensor_publisher.cpp)
  if(TARGET context_gatherer_mock_sensor_test)
    target_link_libraries(context_gatherer_mock_sensor_test
      context_gatherer_mock_sensor)
    ament_target_dependencies(context_gatherer_mock_sensor_test
      rclcpp
      sensor_msgs
      nav_msgs
      geographic_msgs
      image_transport)
    set_tests_properties(context_gatherer_mock_sensor_test PROPERTIES
      ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs")
  endif()

  add_executable(context_gatherer_mock_sensor_node
    test/mock_sensor_publisher_main.cpp)
  target_link_libraries(context_gatherer_mock_sensor_node
    context_gatherer_mock_sensor)
  ament_target_dependencies(context_gatherer_mock_sensor_node
    rclcpp)

  if(COMMAND add_launch_test)
    add_launch_test(test/launch/test_context_gatherer.launch.py
      TARGET context_gatherer_launch_test
      PYTHON_EXECUTABLE /usr/bin/python3)
    if(TARGET context_gatherer_launch_test)
      set_tests_properties(context_gatherer_launch_test PROPERTIES
        ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs;LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$ENV{LD_LIBRARY_PATH}")
    endif()
  endif()

  install(TARGETS context_gatherer_mock_sensor_node
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_targets(export_context_gatherer HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geographic_msgs
  cv_bridge
  image_transport
  nlohmann_json
  std_srvs)

ament_package()
