#include "husky_behavior_tree/husky_behavior_tree.hpp"
#include "husky_behavior_tree/bt_conversions.hpp"
#include "husky_behavior_tree/register_nodes.hpp"

#include <fstream>
#include <filesystem>
#include <thread>

namespace husky_behavior_tree
{

HuskyBehaviorTree::HuskyBehaviorTree(const std::string& node_name)
  : initialized_(false)
  , running_(false)
  , stop_requested_(false)
  , tree_status_(BT::NodeStatus::IDLE)
{
  // Initialize ROS2 node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  node_ = std::make_shared<rclcpp::Node>(node_name);
  
  RCLCPP_INFO(node_->get_logger(), "HuskyBehaviorTree node created: %s", node_name.c_str());
}

HuskyBehaviorTree::~HuskyBehaviorTree()
{
  stop();
  
  // Clean up loggers
  cout_logger_.reset();
  file_logger_.reset();
  
  RCLCPP_INFO(node_->get_logger(), "HuskyBehaviorTree shutting down");
}

bool HuskyBehaviorTree::initialize(const std::string& tree_xml_path)
{
  if (initialized_) {
    RCLCPP_WARN(node_->get_logger(), "Behavior tree already initialized");
    return true;
  }

  // Register all custom nodes
  registerNodes();

  // Create the tree from XML
  if (!createTree(tree_xml_path)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create behavior tree from: %s", tree_xml_path.c_str());
    return false;
  }

  // Initialize loggers
  try {
    cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
    
    // Create log directory if it doesn't exist
    std::filesystem::create_directories("log");
    std::string log_file = "log/bt_trace.btlog";
    file_logger_ = std::make_unique<BT::FileLogger2>(tree_, log_file.c_str());
    
    RCLCPP_INFO(node_->get_logger(), "BehaviorTree loggers initialized (file: %s)", log_file.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize loggers: %s", e.what());
    return false;
  }

  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "Behavior tree initialization complete");
  return true;
}

BT::NodeStatus HuskyBehaviorTree::run(double tick_frequency_hz)
{
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot run: behavior tree not initialized");
    return BT::NodeStatus::FAILURE;
  }

  if (running_) {
    RCLCPP_WARN(node_->get_logger(), "Behavior tree is already running");
    return tree_status_;
  }

  running_ = true;
  stop_requested_ = false;
  
  RCLCPP_INFO(node_->get_logger(), "Starting behavior tree execution at %.2f Hz", tick_frequency_hz);

  // Calculate sleep duration based on tick frequency
  const auto sleep_duration = std::chrono::duration<double>(1.0 / tick_frequency_hz);
  
  while (rclcpp::ok() && running_ && !stop_requested_) {
    auto tick_start = std::chrono::steady_clock::now();
    
    // Tick the behavior tree
    tree_status_ = tick();
    
    // Process ROS callbacks
    rclcpp::spin_some(node_);
    
    // Check if tree has completed (SUCCESS or FAILURE)
    if (tree_status_ == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "Behavior tree completed with SUCCESS");
      break;
    } else if (tree_status_ == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node_->get_logger(), "Behavior tree completed with FAILURE");
      break;
    }
    
    // Sleep to maintain tick rate
    auto tick_end = std::chrono::steady_clock::now();
    auto elapsed = tick_end - tick_start;
    auto remaining = sleep_duration - elapsed;
    
    if (remaining > std::chrono::duration<double>::zero()) {
      std::this_thread::sleep_for(remaining);
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), 
        *node_->get_clock(), 
        5000,
        "Tick took longer than allocated time (%.3f ms > %.3f ms)",
        std::chrono::duration<double, std::milli>(elapsed).count(),
        std::chrono::duration<double, std::milli>(sleep_duration).count()
      );
    }
  }
  
  running_ = false;
  RCLCPP_INFO(node_->get_logger(), "Behavior tree execution stopped");
  
  return tree_status_;
}

BT::NodeStatus HuskyBehaviorTree::tick()
{
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot tick: behavior tree not initialized");
    return BT::NodeStatus::FAILURE;
  }

  try {
    tree_status_ = tree_.tickOnce();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception during tree tick: %s", e.what());
    tree_status_ = BT::NodeStatus::FAILURE;
  }
  
  return tree_status_;
}

void HuskyBehaviorTree::stop()
{
  if (!running_) {
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Stopping behavior tree execution");
  stop_requested_ = true;
  
  // Halt the tree if it's still running
  if (tree_status_ == BT::NodeStatus::RUNNING) {
    try {
      tree_.haltTree();
      RCLCPP_INFO(node_->get_logger(), "Behavior tree halted");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Exception while halting tree: %s", e.what());
    }
  }
  
  running_ = false;
}

bool HuskyBehaviorTree::isRunning() const
{
  return running_;
}

rclcpp::Node::SharedPtr HuskyBehaviorTree::getNode() const
{
  return node_;
}

BT::BehaviorTreeFactory& HuskyBehaviorTree::getFactory()
{
  return factory_;
}

BT::Blackboard::Ptr HuskyBehaviorTree::getBlackboard()
{
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot get blackboard: tree not initialized");
    return nullptr;
  }
  return tree_.rootBlackboard();
}

void HuskyBehaviorTree::registerNodes()
{
  RCLCPP_INFO(node_->get_logger(), "Registering custom behavior tree nodes");
  register_behavior_tree_nodes(factory_);
  RCLCPP_INFO(node_->get_logger(), "All custom nodes registered successfully");
}

bool HuskyBehaviorTree::createTree(const std::string& tree_xml_path)
{
  // Check if file exists
  if (!std::filesystem::exists(tree_xml_path)) {
    RCLCPP_ERROR(node_->get_logger(), "Tree XML file does not exist: %s", tree_xml_path.c_str());
    return false;
  }
  
  try {
    // Load and create the tree from XML file
    tree_ = factory_.createTreeFromFile(tree_xml_path);
    RCLCPP_INFO(node_->get_logger(), "Behavior tree created from: %s", tree_xml_path.c_str());

    // Provide the ROS node on the blackboard for plugin nodes
    if (tree_.rootBlackboard()) {
      tree_.rootBlackboard()->set("node", node_);
    }
    
    // Log tree structure for debugging
    RCLCPP_DEBUG(node_->get_logger(), "Tree structure:");
    //RCLCPP_DEBUG(node_->get_logger(), "%s", BT::WriteTreeToJSON(tree_).c_str());
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create tree from XML: %s", e.what());
    return false;
  }
}

}  // namespace husky_behavior_tree
