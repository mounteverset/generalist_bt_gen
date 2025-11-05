cmake_minimum_required(VERSION 3.8)
project(llm_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_get_python_install_dir(python_destination)

install(FILES
  llm_service_node.py
  prompt_templates.py
  bt_xml_generator.py
  DESTINATION ${python_destination}/${PROJECT_NAME})

install(PROGRAMS
  llm_service_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LLMQuery.srv"
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(llm_interface_prompt_templates_test
    test/test_prompt_templates.py)
  ament_add_pytest_test(llm_interface_bt_xml_generator_test
    test/test_bt_xml_generator.py)
  ament_add_pytest_test(llm_interface_node_test
    test/test_llm_service_node.py)
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
