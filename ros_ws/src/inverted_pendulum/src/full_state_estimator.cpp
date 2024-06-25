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

    // initialize our buffers
    uint buffer_size = 2;
    pos_buffer = boost::circular_buffer<float>(buffer_size);
    theta_buffer = boost::circular_buffer<float>(buffer_size);
    time_buffer = boost::circular_buffer<double>(buffer_size);
  }

private:
  void absolute_state_callback(const geometry_msgs::msg::Pose2D & msg)
  {
    _current_count_theta = msg.theta;
    _current_x_pos = msg.x;

    // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    double now_sec = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1e6);

    RCLCPP_INFO(
      this->get_logger(), 
      "Received absolute state: theta='%f', x_pos='%f' at time '%u'", 
      msg.theta, msg.x, static_cast<uint>(now_sec)
    );

    // Add to deque
    pos_buffer.push_back(_current_x_pos);
    theta_buffer.push_back(_current_count_theta);
    time_buffer.push_back(now_sec);

    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "BUFFER: '%f', '%f', '%f', '%f' at time '%f'", 
    //   pos_buffer[0], pos_buffer[1], time_buffer[0], time_buffer[1], now_sec
    // );

    // Interpolate full state and publish
    // float pos_state = linear_interp(&pos_buffer, &time_buffer, now_sec);
    // float theta_state = linear_interp(&theta_buffer, &time_buffer, now_sec);
    float theta_dot = first_difference(theta_buffer, time_buffer);
    float pos_dot = first_difference(pos_buffer, time_buffer);
   
    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "Theta_dot: '%f', x_dot='%f' at time '%f'", 
    //   theta_dot, pos_dot, now_sec
    // );

    // Send full state
    std_msgs::msg::Float32MultiArray full_state_msg;

    // Set the layout (dimensions and strides)
    // std_msgs::msg::MultiArrayDimension dim;
    // dim.label = "data";
    // dim.size = 4;
    // dim.stride = 1;
    // msg.layout.dim.push_back(dim);

    // Set the data array
    float theta_radians = _current_count_theta * 2 * 3.14159 / 2000;
    float theta_dot_radians = theta_dot * 2 * 3.14159 / 2000;
    full_state_msg.data = {_current_x_pos, pos_dot, theta_radians, theta_dot_radians};

    full_state_publisher_->publish(full_state_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr absolute_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr full_state_publisher_;

  int _current_count_theta;
  float _current_x_pos;

  boost::circular_buffer<float> pos_buffer;
  boost::circular_buffer<float> theta_buffer;
  boost::circular_buffer<double> time_buffer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FullStateEstimator>());
  rclcpp::shutdown();
  return 0;
}
