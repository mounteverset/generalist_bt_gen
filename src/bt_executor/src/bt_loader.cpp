#include "bt_executor/bt_loader.hpp"

#include <filesystem>
#include <stdexcept>

#include "rclcpp/logging.hpp"
#include "yaml-cpp/yaml.h"

namespace bt_executor
{

BTLoader::BTLoader(rclcpp::Node & node, BT::BehaviorTreeFactory & factory)
: logger_{node.get_logger()}, node_{node}, factory_{factory}
{
}

void BTLoader::configure_from_yaml(const std::string & config_path)
{
  if (config_path.empty()) {
    RCLCPP_WARN(logger_, "Loader configuration skipped because config path is empty");
    return;
  }

  const std::filesystem::path path{config_path};
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("BT loader config file does not exist: " + path.string());
  }

  YAML::Node root = YAML::LoadFile(path.string());

  if (root["plugin_libraries"]) {
    register_plugin_libraries(root["plugin_libraries"].as<std::vector<std::string>>());
  }
}

void BTLoader::register_plugin_libraries(const std::vector<std::string> & library_paths)
{
  for (const auto & library_path : library_paths) {
    if (library_path.empty()) {
      continue;
    }

    try {
      factory_.registerFromPlugin(library_path);
      registered_plugins_.push_back(library_path);
      RCLCPP_INFO(logger_, "Registered BT plugin library: %s", library_path.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        logger_, "Failed to register BT plugin library '%s': %s", library_path.c_str(),
        ex.what());
      throw;
    }
  }
}

BT::Tree BTLoader::load_tree(const std::string & xml_path)
{
  if (xml_path.empty()) {
    throw std::invalid_argument("BT XML path must not be empty");
  }

  const std::filesystem::path path{xml_path};
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("BT XML file does not exist: " + path.string());
  }

  RCLCPP_INFO(logger_, "Loading behavior tree from %s", path.c_str());
  return factory_.createTreeFromFile(path);
}

}  // namespace bt_executor
