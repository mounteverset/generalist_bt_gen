#pragma once

#include <memory>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

// BehaviorTree.CPP v4
#include <behaviortree_cpp/bt_factory.h>

namespace husky_behavior_tree
{

// BehaviorTree v4 synchronous action node that calls /read_temp service
// and appends the result to temp_log.txt
class LogTempAction : public BT::SyncActionNode
{
public:
    LogTempAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // No custom ports
        return {};
    }

    BT::NodeStatus tick() override
    {
        using namespace std::chrono_literals;

        if (!ensureNodeAndClient())
        {
            return BT::NodeStatus::FAILURE;
        }

        // Ensure the service is available
        if (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(node_->get_logger(), "Service /read_temp not available");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client_->async_send_request(request);

        // Wait for response
        auto result_code = rclcpp::spin_until_future_complete(node_, future, 3s);
        if (result_code != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Service call to /read_temp failed or timed out");
            return BT::NodeStatus::FAILURE;
        }

        auto response = future.get();
        if (!response->success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Service /read_temp returned failure: %s", response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Append to temp_log.txt with timestamp
        std::ofstream ofs("temp_log.txt", std::ios::app);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open temp_log.txt for writing");
            return BT::NodeStatus::FAILURE;
        }

        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif

        ofs << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << " " << response->message << std::endl;
        ofs.close();

        RCLCPP_INFO(node_->get_logger(), "Logged temperature: %s", response->message.c_str());
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool ensureNodeAndClient()
    {
        if (!node_)
        {
            auto bb = config().blackboard;
            if (!bb)
            {
                RCLCPP_ERROR(rclcpp::get_logger("LogTempAction"), "Blackboard unavailable for node '%s'", name().c_str());
                return false;
            }
            try
            {
                node_ = bb->get<rclcpp::Node::SharedPtr>("node");
            }
            catch (const BT::RuntimeError&)
            {
                RCLCPP_ERROR(rclcpp::get_logger("LogTempAction"),
                             "Failed to access blackboard entry 'node' for '%s'", name().c_str());
                return false;
            }
        }

        if (!node_)
        {
            auto node_logger = rclcpp::get_logger("LogTempAction");
            RCLCPP_ERROR(node_logger, "Null rclcpp::Node provided on blackboard for '%s'", name().c_str());
            return false;
        }

        if (!client_)
        {
            client_ = node_->create_client<std_srvs::srv::Trigger>("/read_temp");
        }

        return true;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

}  // namespace husky_behavior_tree
