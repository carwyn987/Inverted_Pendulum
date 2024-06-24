#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int16.hpp>

using std::placeholders::_1;

class StateEstimator : public rclcpp::Node
{
public:
  StateEstimator()
  : Node("state_estimator")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int16>(
      "encoder_pulse", 10, std::bind(&StateEstimator::encoder_pulse_callback, this, _1));
  }

private:
  void encoder_pulse_callback(const std_msgs::msg::Int16 & msg) const
  {
    // auto now = std::chrono::system_clock::now();
    RCLCPP_INFO(
      this->get_logger(), 
      "Received: '%d'", 
      // "Received: '%d' at time '%u'", 
      msg.data
      // msg.data, static_cast<uint>(now.time_since_epoch().count())
    );
  }
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}
