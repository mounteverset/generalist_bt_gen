#include <chrono>
#include <cstdlib>
#include <memory>

#include "bt_executor/bt_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions options;
    auto server = std::make_shared<bt_executor::LLMTreeServer>(options);

    rclcpp::executors::MultiThreadedExecutor exec(
      rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
    exec.add_node(server->node());
    exec.spin();
    exec.remove_node(server->node());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("bt_executor_node"), "Unhandled exception: %s", ex.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("bt_executor_node"), "Unhandled non-standard exception");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
