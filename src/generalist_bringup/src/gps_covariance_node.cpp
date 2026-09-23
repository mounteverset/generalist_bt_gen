#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GpsCovarianceNode : public rclcpp::Node
{
public:
  GpsCovarianceNode()
  : Node("gps_covariance_node")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/gps/fix_raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/gps/fix");
    const auto position_stddev_m = declare_parameter<double>("position_stddev_m", 0.2);
    if (!std::isfinite(position_stddev_m) || position_stddev_m < 0.0) {
      throw std::invalid_argument("position_stddev_m must be finite and non-negative");
    }
    position_variance_m2_ = position_stddev_m * position_stddev_m;

    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      output_topic, rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr input) {
        auto output = *input;
        output.position_covariance = {
          position_variance_m2_, 0.0, 0.0,
          0.0, position_variance_m2_, 0.0,
          0.0, 0.0, position_variance_m2_};
        output.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        publisher_->publish(output);
      });

    RCLCPP_INFO(
      get_logger(), "Adding %.2f m GPS position uncertainty: %s -> %s",
      position_stddev_m, input_topic.c_str(), output_topic.c_str());
  }

private:
  double position_variance_m2_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsCovarianceNode>());
  rclcpp::shutdown();
  return 0;
}
