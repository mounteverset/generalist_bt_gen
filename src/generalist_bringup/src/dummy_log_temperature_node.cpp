#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class DummyLogTemperatureNode : public rclcpp::Node
{
public:
  DummyLogTemperatureNode()
  : Node("dummy_log_temperature_node")
  {
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "/log_temperature",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        (void)this;
        response->success = true;
        response->message = "Dummy log_temperature service: success";
      });

    RCLCPP_INFO(this->get_logger(), "Dummy /log_temperature Trigger service ready.");
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyLogTemperatureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

