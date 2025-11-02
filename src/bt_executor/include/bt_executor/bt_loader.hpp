#ifndef BT_EXECUTOR__BT_LOADER_HPP_
#define BT_EXECUTOR__BT_LOADER_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace bt_executor
{

class BTLoader
{
public:
  BTLoader(rclcpp::Node & node, BT::BehaviorTreeFactory & factory);

  void configure_from_yaml(const std::string & config_path);

  void register_plugin_libraries(const std::vector<std::string> & library_paths);

  BT::Tree load_tree(const std::string & xml_path);

private:
  rclcpp::Logger logger_;
  rclcpp::Node & node_;
  BT::BehaviorTreeFactory & factory_;
  std::vector<std::string> registered_plugins_;
};
}  // namespace bt_executor

#endif  // BT_EXECUTOR__BT_LOADER_HPP_
