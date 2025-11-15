#include "bt_executor/generalist_bt_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto server = std::make_shared<bt_executor::GeneralistBehaviorTreeServer>(options);
  rclcpp::spin(server->node());
  rclcpp::shutdown();
  return 0;
}
