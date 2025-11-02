#include <cstdlib>
#include <memory>

#include "bt_executor/bt_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto executor = std::make_shared<bt_executor::BTExecutor>();
    executor->configure();
    executor->start();
    rclcpp::spin(executor);
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
