#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

class MockGpsFixPublisherNode : public rclcpp::Node
{
public:
  MockGpsFixPublisherNode()
  : Node("mock_gps_fix_publisher_node")
  {
    topic_name_ = declare_parameter<std::string>("topic_name", "/gps/fix");
    frame_id_ = declare_parameter<std::string>("frame_id", "gps_link");
    latitude_deg_ = declare_parameter<double>("latitude_deg", 48.20286111111111);
    longitude_deg_ = declare_parameter<double>("longitude_deg", 11.64486111111111);
    altitude_m_ = declare_parameter<double>("altitude_m", 0.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);
    position_covariance_diagonal_m2_ = declare_parameter<double>(
      "position_covariance_diagonal_m2", 4.0);
    status_code_ = declare_parameter<int>("status_code", sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    service_code_ = declare_parameter<int>(
      "service_code",
      sensor_msgs::msg::NavSatStatus::SERVICE_GPS | sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS);

    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(topic_name_, 10);

    const double safe_rate_hz = publish_rate_hz_ > 0.0 ? publish_rate_hz_ : 1.0;
    const auto publish_period = std::chrono::duration<double>(1.0 / safe_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(publish_period),
      std::bind(&MockGpsFixPublisherNode::publish_fix, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing mock GPS fixes to %s at %.2f Hz (lat=%.8f, lon=%.8f, alt=%.2f m)",
      topic_name_.c_str(),
      safe_rate_hz,
      latitude_deg_,
      longitude_deg_,
      altitude_m_);
  }

private:
  void publish_fix()
  {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.status.status = static_cast<int8_t>(status_code_);
    msg.status.service = static_cast<uint16_t>(service_code_);
    msg.latitude = latitude_deg_;
    msg.longitude = longitude_deg_;
    msg.altitude = altitude_m_;
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg.position_covariance = {
      position_covariance_diagonal_m2_, 0.0, 0.0,
      0.0, position_covariance_diagonal_m2_, 0.0,
      0.0, 0.0, position_covariance_diagonal_m2_};
    publisher_->publish(msg);
  }

  std::string topic_name_;
  std::string frame_id_;
  double latitude_deg_;
  double longitude_deg_;
  double altitude_m_;
  double publish_rate_hz_;
  double position_covariance_diagonal_m2_;
  int status_code_;
  int service_code_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MockGpsFixPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
