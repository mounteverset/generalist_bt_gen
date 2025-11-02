#include "bt_executor/bt_executor.hpp"

#include <functional>
#include <stdexcept>

namespace bt_executor
{

namespace
{
constexpr char kNodeName[] = "bt_executor";
constexpr char kContextServiceName[] = "context_gatherer/snapshot";
constexpr char kUpdaterServiceName[] = "bt_updater/reload_tree";
}  // namespace

BTExecutor::BTExecutor(const rclcpp::NodeOptions & options)
: rclcpp::Node(kNodeName, options)
{
  declare_parameters();
}

BTExecutor::~BTExecutor()
{
  stop();
}

void BTExecutor::configure()
{
  read_parameters();
  initialize_components();
  initialize_services();
  create_tree();
  configured_.store(true);

  if (config_.auto_start) {
    start();
  }
}

void BTExecutor::start()
{
  if (!configured_.load()) {
    configure();
    return;
  }

  if (running_.exchange(true)) {
    return;
  }

  schedule_tick_timer();
}

void BTExecutor::stop()
{
  running_.store(false);
  if (tick_timer_) {
    tick_timer_->cancel();
    tick_timer_.reset();
  }
}

void BTExecutor::tick()
{
  if (!running_.load()) {
    return;
  }

  if (!tree_.rootNode()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000.0, "Behavior tree root node not set, skipping tick");
    return;
  }

  tree_.tickOnce();
}

bool BTExecutor::is_running() const
{
  return running_.load();
}

const ExecutorConfig & BTExecutor::config() const
{
  return config_;
}

BT::Tree & BTExecutor::tree()
{
  return tree_;
}

BT::Blackboard::Ptr BTExecutor::blackboard()
{
  return blackboard_;
}

BTLoader & BTExecutor::loader()
{
  if (!loader_) {
    throw std::runtime_error("BTLoader is not initialised");
  }

  return *loader_;
}

BTMonitor & BTExecutor::monitor()
{
  if (!monitor_) {
    throw std::runtime_error("BTMonitor is not initialised");
  }

  return *monitor_;
}

BTFailureHandler & BTExecutor::failure_handler()
{
  if (!failure_handler_) {
    throw std::runtime_error("BTFailureHandler is not initialised");
  }

  return *failure_handler_;
}

rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr BTExecutor::context_request_client() const
{
  return context_client_;
}

rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr BTExecutor::update_tree_client() const
{
  return updater_client_;
}

void BTExecutor::declare_parameters()
{
  config_.tick_period = std::chrono::milliseconds(100);
  config_.auto_start = true;

  this->declare_parameter<std::string>("tree_xml_path", config_.tree_xml_path);
  this->declare_parameter<std::string>("config_yaml_path", config_.config_yaml_path);
  this->declare_parameter<int>("tick_period_ms", static_cast<int>(config_.tick_period.count()));
  this->declare_parameter<bool>("auto_start", config_.auto_start);
}

void BTExecutor::read_parameters()
{
  config_.tree_xml_path = this->get_parameter("tree_xml_path").as_string();
  config_.config_yaml_path = this->get_parameter("config_yaml_path").as_string();
  config_.tick_period = std::chrono::milliseconds(
    this->get_parameter("tick_period_ms").as_int());
  config_.auto_start = this->get_parameter("auto_start").as_bool();
}

void BTExecutor::initialize_components()
{
  // Component instances will be created once their implementations are available.
}

void BTExecutor::initialize_services()
{
  context_client_ = this->create_client<std_srvs::srv::Trigger>(kContextServiceName);
  updater_client_ = this->create_client<std_srvs::srv::Trigger>(kUpdaterServiceName);
}

void BTExecutor::create_tree()
{
  tree_ = BT::Tree();
  blackboard_ = tree_.rootBlackboard();
}

void BTExecutor::schedule_tick_timer()
{
  tick_timer_ = this->create_wall_timer(
    config_.tick_period, std::bind(&BTExecutor::tick, this));
}

void BTExecutor::teardown_tree()
{
  tree_ = BT::Tree();
  blackboard_.reset();
}
}  // namespace bt_executor
