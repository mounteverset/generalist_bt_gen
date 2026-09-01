#include <ament_index_cpp/get_package_prefix.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/bt_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const std::string xml{
      std::istreambuf_iterator<char>(std::cin),
      std::istreambuf_iterator<char>()};
    if (xml.empty()) {
      throw std::runtime_error("No XML was supplied on stdin.");
    }

    BT::BehaviorTreeFactory factory;
    auto node = std::make_shared<rclcpp::Node>("bt_factory_check");
    BT::RosNodeParams params;
    params.nh = node;
    params.server_timeout = std::chrono::milliseconds(100);
    params.wait_for_server_timeout = params.server_timeout;

    auto plugin_path =
      std::filesystem::path(ament_index_cpp::get_package_prefix("robot_actions")) /
      "lib" / "librobot_actions.so";
    if (argc == 2) {
      plugin_path = argv[1];
    }
    if (!std::filesystem::is_regular_file(plugin_path)) {
      throw std::runtime_error(
              "robot_actions plugin not found: " + plugin_path.string());
    }

    BT::LoadPlugin(factory, plugin_path, params);
    auto blackboard = BT::Blackboard::create();
    auto tree = factory.createTreeFromText(xml, blackboard);
    (void)tree;
    std::cout << "BT.CPP factory load passed" << std::endl;
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "BT.CPP factory load failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
