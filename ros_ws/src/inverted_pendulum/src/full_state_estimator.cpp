#include <functional>
#include <memory>
#include <boost/circular_buffer.hpp>

#include "inverted_pendulum/interpolate.hpp"

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using std::placeholders::_1;

class FullStateEstimator : public rclcpp::Node
{
public:
  FullStateEstimator()
  : Node("full_state_estimator")
  {
    absolute_state_subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "absolute_state", 10, std::bind(&FullStateEstimator::absolute_state_callback, this, _1));
    full_state_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("full_state", 10);

    // Initialize state buffer
    uint buffer_size = 6;
    state_buffer = boost::circular_buffer<float>(buffer_size);
  }

private:
  void absolute_state_callback(const geometry_msgs::msg::Pose2D & msg)
  {
    _current_count_theta = static_cast<float>(msg.theta);
    _current_x_pos = msg.x;

    // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    double now_sec = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1e6);

    // Send full state
    std_msgs::msg::Float32MultiArray full_state_msg;

    // Currently the data is 4000 and -4000 at theta = pi, and 0 at theta = 0. Convert to 0 and 4000 at 0, and 2000 at pi.
    if (_current_count_theta < 0){
      _current_count_theta += 8000;
    }
    // now _current_count_theta is between 0 and 4000
    float theta_radians = _current_count_theta * 2 * M_PI / 8000;

    // Combine the saved state buffer with the current state
    state_buffer.push_back(_current_x_pos);
    state_buffer.push_back(theta_radians);

    full_state_msg.data = {state_buffer[0], state_buffer[1], state_buffer[2], state_buffer[3], state_buffer[4], state_buffer[5]}; // convert_to_float32_array(state_buffer);
    
    RCLCPP_INFO(
      this->get_logger(), 
      "state_buffer='%f %f %f %f %f %f' at time '%f'", 
      full_state_msg.data[0], full_state_msg.data[1], full_state_msg.data[2], full_state_msg.data[3], full_state_msg.data[4], full_state_msg.data[5], now_sec
    );

    full_state_publisher_->publish(full_state_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr absolute_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr full_state_publisher_;

  float _current_count_theta;
  float _current_x_pos;

  boost::circular_buffer<float> state_buffer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FullStateEstimator>());
  rclcpp::shutdown();
  return 0;
}
