#include <functional>
#include <memory>
#include <cmath>

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
    if (first_encoder_pulse)
    {
      publish_absolute_state();
      first_encoder_pulse = false;
    }
    
  }

  void stepper_pulse_callback(const std_msgs::msg::Float32 & msg)
  {
    // Remember: msg.data from the stepper is a step, not absolute. We are tracking absolute x_pos
    // msg.data is the step size, which can be either 1, 1/2, 1/4, 1/8, 1/16, or 1/32. Compute the equivalent distance:

    // float applied_move = get_applied_move(msg.data); // in number of steps
    // applied_move *= 1.8 / 360.0; // steps -> degrees -> now into percentage of a full rotation
    // float radius = 0.028 * 16; // 4.8 cm :: 248 full steps = 1/2 distance 
    // applied_move *= 2 * M_PI * radius; // now to distance
    _x_pos += msg.data; // Convert to (?))

    // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "!!!!!!!!!!!!!!!!!!!!!!!!! Received stepper pulse: '%f' at time '%u' for a new count of %f", 
    //   msg.data, static_cast<uint>(now_sec), _x_pos
    // );

    // delay 1 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    publish_absolute_state();
  }

  float get_applied_move(float move) {
    // Determine the direction
    bool direction = (move > 0); // true for positive, false for negative
    move = std::abs(move); // Make the move positive for step size calculation

    // Determine the step size
    float step_size = 0.0;
    if (move >= 1.0) {
        step_size = 1.0;
    } else if (move <= 1.0 / 32.0) {
        step_size = 1.0 / 32.0;
    } else {
        // Define cutoffs
        float cutoffs[] = {1.0, 0.75, 0.375, 0.1875, 0.09375, 0.046875, 0.03125}; // These are the midpoints between step sizes

        // Find the appropriate index for the step size
        for (int i = 0; i < 6; ++i) {
            if (move < cutoffs[i] && move >= cutoffs[i + 1]) {
                switch (i) {
                    case 0: step_size = 1.0 / 2.0; break;
                    case 1: step_size = 1.0 / 4.0; break;
                    case 2: step_size = 1.0 / 8.0; break;
                    case 3: step_size = 1.0 / 16.0; break;
                    case 4: step_size = 1.0 / 32.0; break;
                }
                break;
            }
        }
    }

    // Update the position based on direction
    if (direction) {
        return step_size;
    } else {
        return -1 * step_size;
    }
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

  bool first_encoder_pulse = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}
