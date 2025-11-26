#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>

#include "gen_bt_interfaces/action/gather_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class ContextGathererNode : public rclcpp::Node
{
public:
  using GatherContext = gen_bt_interfaces::action::GatherContext;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GatherContext>;

  ContextGathererNode()
  : Node("context_gatherer")
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "/context_gatherer/gather");
    output_directory_ = this->declare_parameter<std::string>("output_directory", "/tmp/context_gatherer");
    default_timeout_sec_ = this->declare_parameter<double>("default_timeout_sec", 10.0);
    debug_logging_ = this->declare_parameter<bool>("enable_debug_logging", true);
    default_requirements_str_ = this->declare_parameter<std::string>(
      "default_requirements", "ROBOT_POSE,RGB_IMAGE");
    default_requirements_ = parse_requirements(default_requirements_str_);

    RCLCPP_INFO(
      get_logger(),
      "Starting context_gatherer (action=%s, output_dir=%s, default_timeout=%.2fs)",
      action_name_.c_str(), output_directory_.c_str(), default_timeout_sec_);

    action_server_ = rclcpp_action::create_server<GatherContext>(
      this,
      action_name_,
      std::bind(&ContextGathererNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ContextGathererNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ContextGathererNode::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<GatherContext>::SharedPtr action_server_;
  std::string action_name_;
  std::string output_directory_;
  double default_timeout_sec_;
  bool debug_logging_;
  std::string default_requirements_str_;
  std::vector<std::string> default_requirements_;

  std::vector<std::string> parse_requirements(const std::string & csv)
  {
    std::vector<std::string> result;
    std::string current;
    for (char c : csv) {
      if (c == ',' || c == ';') {
        if (!current.empty()) {
          trim_and_push(result, current);
        }
        current.clear();
      } else {
        current.push_back(c);
      }
    }
    if (!current.empty()) {
      trim_and_push(result, current);
    }
    return result;
  }

  void trim_and_push(std::vector<std::string> & vec, std::string & token)
  {
    auto start = token.find_first_not_of(" \t");
    auto end = token.find_last_not_of(" \t");
    if (start == std::string::npos || end == std::string::npos) {
      return;
    }
    std::string trimmed = token.substr(start, end - start + 1);
    if (!trimmed.empty()) {
      vec.push_back(trimmed);
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GatherContext::Goal> goal)
  {
    if (goal->timeout_sec <= 0.0) {
      RCLCPP_WARN(get_logger(), "Goal timeout not set; using default %.2fs", default_timeout_sec_);
    }
    RCLCPP_INFO(
      get_logger(), "Received GatherContext goal (session=%s, subtree=%s, reqs=%zu)",
      goal->session_id.c_str(), goal->subtree_id.c_str(), goal->context_requirements.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(
      get_logger(), "Cancel requested for session=%s",
      goal_handle->get_goal()->session_id.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&ContextGathererNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GatherContext::Result>();
    const double timeout_sec = goal->timeout_sec > 0.0 ? goal->timeout_sec : default_timeout_sec_;
    const auto start = now_seconds();

    std::vector<std::string> requirements = goal->context_requirements.empty()
      ? default_requirements_
      : goal->context_requirements;

    auto feedback = std::make_shared<GatherContext::Feedback>();
    feedback->stage = "START";
    feedback->detail = "Collecting context requirements.";
    goal_handle->publish_feedback(feedback);

    for (const auto & req : requirements) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Goal canceled during collection.";
        goal_handle->canceled(result);
        return;
      }
      feedback->stage = "COLLECT";
      feedback->detail = req;
      goal_handle->publish_feedback(feedback);
      if (elapsed_seconds(start) > timeout_sec) {
        result->success = false;
        result->message = "Timeout while collecting context.";
        goal_handle->abort(result);
        return;
      }
      // Placeholder for actual data acquisition per requirement.
      rclcpp::sleep_for(50ms);
    }

    feedback->stage = "FINALIZE";
    feedback->detail = "Building snapshot.";
    goal_handle->publish_feedback(feedback);

    result->context_json = build_stub_json(goal->session_id, goal->subtree_id, requirements);
    result->attachment_uris.clear();
    result->success = true;
    result->message = "Stub snapshot generated.";
    goal_handle->succeed(result);

    RCLCPP_INFO(
      get_logger(),
      "Context gathered for session=%s subtree=%s (reqs=%zu)",
      goal->session_id.c_str(), goal->subtree_id.c_str(), requirements.size());
  }

  double now_seconds() const
  {
    return this->now().seconds();
  }

  double elapsed_seconds(double start_sec) const
  {
    return now_seconds() - start_sec;
  }

  std::string build_stub_json(
    const std::string & session_id,
    const std::string & subtree_id,
    const std::vector<std::string> & requirements) const
  {
    std::ostringstream oss;
    oss << "{";
    oss << "\"session_id\":\"" << session_id << "\",";
    oss << "\"subtree_id\":\"" << subtree_id << "\",";
    oss << "\"timestamp_sec\":" << now_seconds() << ",";
    oss << "\"requirements\":[";
    for (size_t i = 0; i < requirements.size(); ++i) {
      oss << "\"" << requirements[i] << "\"";
      if (i + 1 < requirements.size()) {
        oss << ",";
      }
    }
    oss << "],";
    oss << "\"note\":\"Stub context_gatherer; replace with sensor integrations.\",";
    oss << "\"attachments\":[]";
    oss << "}";
    return oss.str();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContextGathererNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
