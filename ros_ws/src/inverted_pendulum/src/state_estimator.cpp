#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

using std::placeholders::_1;

class StateEstimator : public rclcpp::Node
{
public:
  StateEstimator()
  : Node("state_estimator")
  {
    encoder_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
      "encoder_pulse", 10, std::bind(&StateEstimator::encoder_pulse_callback, this, _1));
    stepper_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "inference", 10, std::bind(&StateEstimator::stepper_pulse_callback, this, _1));
    absolute_state_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("absolute_state", 10);
  }

private:
  void encoder_pulse_callback(const std_msgs::msg::Int16 & msg)
  {
    // _count_theta += msg.data; // += if we are going to use the encoder as a counter
    _count_theta = msg.data; // = if we are going to use the encoder as an absolute value

    // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "Received encoder pulse: '%d' at time '%u' for a new count of %d", 
    //   msg.data, static_cast<uint>(now_sec), _count_theta
    // );

    // Need this to start the process (get any stepper readings)
    publish_absolute_state();
  }

  void stepper_pulse_callback(const std_msgs::msg::Float32 & msg)
  {
    // Remember: msg.data from the stepper is a step, not absolute. We are tracking absolute x_pos
    _x_pos += msg.data * 100 / 1000.0; // Convert to (?))

    // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "!!!!!!!!!!!!!!!!!!!!!!!!! Received stepper pulse: '%f' at time '%u' for a new count of %f", 
    //   msg.data, static_cast<uint>(now_sec), _x_pos
    // );

    // publish_absolute_state();
  }

  void publish_absolute_state()
  {
    geometry_msgs::msg::Pose2D msg;
    msg.x = _x_pos;
    msg.y = 0.0; // Unused
    msg.theta = _count_theta;
    absolute_state_publisher_->publish(msg);
  }

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr encoder_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stepper_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr absolute_state_publisher_;

  int _count_theta = 0;
  float _x_pos = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}
