#include "husky_behavior_tree/register_nodes.hpp"

#include <memory>

#include "husky_behavior_tree/drive_to.hpp"
#include "husky_behavior_tree/log_temp.hpp"

namespace husky_behavior_tree
{

void register_behavior_tree_nodes(BT::BehaviorTreeFactory& factory)
{
  BT::NodeBuilder drive_to_builder = [](const std::string& name, const BT::NodeConfig& config)
  {
    return std::make_unique<DriveTo>(name, config);
  };
  factory.registerBuilder<DriveTo>("DriveTo", drive_to_builder);

  BT::NodeBuilder log_temp_builder = [](const std::string& name, const BT::NodeConfig& config)
  {
    return std::make_unique<LogTempAction>(name, config);
  };
  factory.registerBuilder<LogTempAction>("LogTemp", log_temp_builder);
}

}  // namespace husky_behavior_tree
