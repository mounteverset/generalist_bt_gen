#ifndef HUSKY_BEHAVIOR_TREE_HPP
#define HUSKY_BEHAVIOR_TREE_HPP

#include <memory>
#include <string>
#include <chrono>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "rclcpp/rclcpp.hpp"

namespace husky_behavior_tree
{

class HuskyBehaviorTree
{
public:
  /**
   * @brief Constructor
   * @param node_name Name of the ROS2 node
   */
  explicit HuskyBehaviorTree(const std::string& node_name = "husky_behavior_tree");

  /**
   * @brief Destructor
   */
  ~HuskyBehaviorTree();

  /**
   * @brief Initialize the behavior tree
   * @param tree_xml_path Path to the XML file defining the behavior tree
   * @return true if initialization was successful, false otherwise
   */
  bool initialize(const std::string& tree_xml_path);

  /**
   * @brief Main run loop - ticks the behavior tree at specified frequency
   * @param tick_frequency_hz Frequency at which to tick the tree (default: 10 Hz)
   * @return Final status of the behavior tree execution
   */
  BT::NodeStatus run(double tick_frequency_hz = 10.0);

  /**
   * @brief Tick the behavior tree once
   * @return Status of the tree after ticking
   */
  BT::NodeStatus tick();

  /**
   * @brief Stop the behavior tree execution
   */
  void stop();

  /**
   * @brief Check if the behavior tree is running
   * @return true if running, false otherwise
   */
  bool isRunning() const;

  /**
   * @brief Get the ROS2 node
   * @return Shared pointer to the ROS2 node
   */
  rclcpp::Node::SharedPtr getNode() const;

  /**
   * @brief Get the BehaviorTree factory
   * @return Reference to the BT factory
   */
  BT::BehaviorTreeFactory& getFactory();

  /**
   * @brief Get the blackboard of the behavior tree
   * @return Shared pointer to the blackboard
   */
  BT::Blackboard::Ptr getBlackboard();

protected:
  /**
   * @brief Register custom nodes with the BehaviorTree factory
   */
  void registerNodes();

  /**
   * @brief Create the behavior tree from XML
   * @param tree_xml_path Path to the XML file
   * @return true if tree was created successfully, false otherwise
   */
  bool createTree(const std::string& tree_xml_path);

private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  // BehaviorTree.CPP factory and tree
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;

  // Loggers
  std::unique_ptr<BT::StdCoutLogger> cout_logger_;
  std::unique_ptr<BT::FileLogger2> file_logger_;

  // State flags
  bool initialized_;
  bool running_;
  bool stop_requested_;

  // Tree execution status
  BT::NodeStatus tree_status_;
};

}  // namespace husky_behavior_tree

#endif  // HUSKY_BEHAVIOR_TREE_HPP
