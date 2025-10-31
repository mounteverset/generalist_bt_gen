#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <optional>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace husky_behavior_tree
{

// Basic BT.CPP v4 async action that sends a NavigateToPose goal using a ROS 2 action client.
// - Input Port: "goal_pose" (geometry_msgs::msg::PoseStamped)
// - The action name (topic) is read from a ROS 2 parameter "action_name" during construction.
// - onStart() sends the goal; onRunning() checks for the result; onHalted() cancels if needed.
class DriveTo : public BT::StatefulActionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle     = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using WrappedResult  = GoalHandle::WrappedResult;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Target PoseStamped goal")
        };
    }

    // Recommended: pass an rclcpp::Node::SharedPtr explicitly when registering this node.
    // Example:
    //   factory.registerNodeType<husky_behavior_tree::DriveTo>("DriveTo", node);
    DriveTo(const std::string& name,
                    const BT::NodeConfig& config,
                    const rclcpp::Node::SharedPtr& node)
        : BT::StatefulActionNode(name, config),
            node_(node)
    {
        if (!node_) {
            throw std::runtime_error("DriveTo requires a valid rclcpp::Node");
        }

        // Get action name from ROS 2 parameter during construction
        // Default aligns with Nav2's NavigateToPose action server name.
        if (!node_->has_parameter("action_name")) {
            node_->declare_parameter<std::string>("action_name", "navigate_to_pose");
        }
        node_->get_parameter("action_name", action_name_);

        client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name_);

        // Pre-configure callbacks
        send_goal_options_.goal_response_callback =
            [this](typename GoalHandle::SharedPtr gh) {
                goal_handle_ = gh;
                if (!gh) {
                    RCLCPP_ERROR(node_->get_logger(), "[DriveTo] Goal was rejected by server '%s'", action_name_.c_str());
                    goal_rejected_ = true;
                } else {
                    RCLCPP_DEBUG(node_->get_logger(), "[DriveTo] Goal accepted by server '%s'", action_name_.c_str());
                }
            };

        send_goal_options_.feedback_callback =
            [this](GoalHandle::SharedPtr /*handle*/,
                         const std::shared_ptr<const NavigateToPose::Feedback> /*feedback*/) {
                // Feedback not used here, but could be logged or published.
            };

        send_goal_options_.result_callback =
            [this](const WrappedResult& result) {
                last_result_ = result;
                result_received_ = true;
                RCLCPP_DEBUG(node_->get_logger(), "[DriveTo] Result received from '%s' (code=%d)",
                                         action_name_.c_str(), static_cast<int>(result.code));
            };
    }

    BT::NodeStatus onStart() override
    {
        // Reset state
        result_received_ = false;
        goal_rejected_ = false;
        last_result_.reset();
        goal_handle_.reset();

        // Read goal from port
        geometry_msgs::msg::PoseStamped goal_pose;
        if (!getInput("goal_pose", goal_pose)) {
            RCLCPP_ERROR(node_->get_logger(), "[DriveTo] Missing input port 'goal_pose'");
            return BT::NodeStatus::FAILURE;
        }

        // Non-blocking check for server availability; keep trying while RUNNING
        if (!client_->wait_for_action_server(std::chrono::seconds(0))) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                                     "[DriveTo] Action server '%s' not available yet", action_name_.c_str());
            // Try again on next tick
            return BT::NodeStatus::RUNNING;
        }

        // Construct and send goal
        NavigateToPose::Goal goal_msg;
        goal_msg.pose = std::move(goal_pose);

        try {
            (void)client_->async_send_goal(goal_msg, send_goal_options_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[DriveTo] async_send_goal exception: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // If the goal was immediately rejected
        if (goal_rejected_) {
            return BT::NodeStatus::FAILURE;
        }

        // If we already have the result, decide status
        if (result_received_) {
            if (!last_result_.has_value()) {
                return BT::NodeStatus::FAILURE;
            }
            switch (last_result_->code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    return BT::NodeStatus::SUCCESS;
                case rclcpp_action::ResultCode::ABORTED:
                case rclcpp_action::ResultCode::CANCELED:
                default:
                    return BT::NodeStatus::FAILURE;
            }
        }

        // Keep running until result arrives
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        if (goal_handle_) {
            try {
                (void)client_->async_cancel_goal(goal_handle_);
                RCLCPP_DEBUG(node_->get_logger(), "[DriveTo] Goal canceled on halt");
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(), "[DriveTo] async_cancel_goal exception: %s", e.what());
            }
        }

        // Reset state
        result_received_ = false;
        goal_rejected_ = false;
        last_result_.reset();
        goal_handle_.reset();
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string action_name_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    typename GoalHandle::SharedPtr goal_handle_;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options_;

    std::optional<WrappedResult> last_result_;
    bool result_received_{false};
    bool goal_rejected_{false};
};

}  // namespace husky_behavior_tree

// Helper to register with BT factory when you have a node available.
inline void RegisterDriveTo(BT::BehaviorTreeFactory& factory, const rclcpp::Node::SharedPtr& node)
{
    factory.registerNodeType<husky_behavior_tree::DriveTo>("DriveTo", node);
}