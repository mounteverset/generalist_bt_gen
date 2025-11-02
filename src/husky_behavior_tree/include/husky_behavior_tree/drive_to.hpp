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
// - The rclcpp::Node is retrieved from the blackboard entry "node".
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

    DriveTo(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
    }

    BT::NodeStatus onStart() override
    {
        // Reset state
        result_received_ = false;
        goal_rejected_ = false;
        last_result_.reset();
        goal_handle_.reset();

        if (!ensureNodeAndClient()) {
            return BT::NodeStatus::FAILURE;
        }

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
        if (!node_) {
            return BT::NodeStatus::FAILURE;
        }

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
                if (node_) {
                    RCLCPP_DEBUG(node_->get_logger(), "[DriveTo] Goal canceled on halt");
                }
            } catch (const std::exception& e) {
                if (node_) {
                    RCLCPP_WARN(node_->get_logger(), "[DriveTo] async_cancel_goal exception: %s", e.what());
                }
            }
        }

        // Reset state
        result_received_ = false;
        goal_rejected_ = false;
        last_result_.reset();
        goal_handle_.reset();
    }

private:
    bool ensureNodeAndClient()
    {
        if (!node_) {
            auto bb = config().blackboard;
            if (!bb) {
                RCLCPP_ERROR(rclcpp::get_logger("DriveTo"), "Blackboard unavailable for node '%s'", name().c_str());
                return false;
            }
            try {
                node_ = bb->get<rclcpp::Node::SharedPtr>("node");
            } catch (const BT::RuntimeError& e) {
                (void)e;
                RCLCPP_ERROR(rclcpp::get_logger("DriveTo"),
                             "Failed to access blackboard entry 'node' for '%s'", name().c_str());
                return false;
            }
            if (!node_) {
                RCLCPP_ERROR(rclcpp::get_logger("DriveTo"),
                             "Null rclcpp::Node provided on blackboard for '%s'", name().c_str());
                return false;
            }

            if (!node_->has_parameter("action_name")) {
                node_->declare_parameter<std::string>("action_name", action_name_);
            }
            node_->get_parameter("action_name", action_name_);
        }

        if (!client_) {
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
                    if (node_) {
                        RCLCPP_DEBUG(node_->get_logger(), "[DriveTo] Result received from '%s' (code=%d)",
                                     action_name_.c_str(), static_cast<int>(result.code));
                    }
                };
        }

        return true;
    }

    rclcpp::Node::SharedPtr node_;
    std::string action_name_{"navigate_to_pose"};
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    typename GoalHandle::SharedPtr goal_handle_;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options_;

    std::optional<WrappedResult> last_result_;
    bool result_received_{false};
    bool goal_rejected_{false};
};

}  // namespace husky_behavior_tree
