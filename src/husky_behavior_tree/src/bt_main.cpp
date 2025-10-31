#include "husky_behavior_tree/husky_behavior_tree.hpp"
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <memory>

// Global pointer for signal handling
std::shared_ptr<husky_behavior_tree::HuskyBehaviorTree> g_bt_instance = nullptr;

void signalHandler(int signum)
{
  RCLCPP_INFO(rclcpp::get_logger("bt_main"), "Interrupt signal (%d) received, stopping behavior tree...", signum);
  
  if (g_bt_instance) {
    g_bt_instance->stop();
  }
  
  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Register signal handler for graceful shutdown
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  
  try {
    // Create the behavior tree instance
    g_bt_instance = std::make_shared<husky_behavior_tree::HuskyBehaviorTree>("husky_bt_node");
    
    // Get ROS2 parameters
    auto node = g_bt_instance->getNode();
    
    // Declare and get the XML file path parameter
    std::string tree_xml_path;
    if (!node->has_parameter("tree_xml_file")) {
      node->declare_parameter<std::string>("tree_xml_file", "");
    }
    node->get_parameter("tree_xml_file", tree_xml_path);
    
    // Check if XML path was provided
    if (tree_xml_path.empty()) {
      RCLCPP_ERROR(node->get_logger(), "No behavior tree XML file specified!");
      RCLCPP_ERROR(node->get_logger(), "Please provide the 'tree_xml_file' parameter.");
      RCLCPP_ERROR(node->get_logger(), "Example: ros2 run husky_behavior_tree bt_main --ros-args -p tree_xml_file:=/path/to/tree.xml");
      rclcpp::shutdown();
      return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", tree_xml_path.c_str());
    
    // Initialize the behavior tree
    if (!g_bt_instance->initialize(tree_xml_path)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize behavior tree");
      rclcpp::shutdown();
      return 1;
    }
    
    // Declare and get tick frequency parameter
    double tick_frequency_hz = 10.0;
    if (!node->has_parameter("tick_frequency")) {
      node->declare_parameter<double>("tick_frequency", tick_frequency_hz);
    }
    node->get_parameter("tick_frequency", tick_frequency_hz);
    
    RCLCPP_INFO(node->get_logger(), "Starting behavior tree execution at %.2f Hz", tick_frequency_hz);
    
    // Run the behavior tree (this blocks until completion or stop)
    BT::NodeStatus final_status = g_bt_instance->run(tick_frequency_hz);
    
    // Report final status
    switch (final_status) {
      case BT::NodeStatus::SUCCESS:
        RCLCPP_INFO(node->get_logger(), "Behavior tree finished with SUCCESS");
        break;
      case BT::NodeStatus::FAILURE:
        RCLCPP_WARN(node->get_logger(), "Behavior tree finished with FAILURE");
        break;
      case BT::NodeStatus::RUNNING:
        RCLCPP_INFO(node->get_logger(), "Behavior tree stopped while RUNNING");
        break;
      default:
        RCLCPP_WARN(node->get_logger(), "Behavior tree finished with IDLE/SKIPPED status");
        break;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("bt_main"), "Exception caught: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  // Clean shutdown
  g_bt_instance.reset();
  rclcpp::shutdown();
  
  RCLCPP_INFO(rclcpp::get_logger("bt_main"), "Behavior tree node shutdown complete");
  
  return 0;
}
