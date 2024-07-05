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
    uint buffer_size = 5;
    pos_buffer = boost::circular_buffer<float>(buffer_size);
    time_buffer = boost::circular_buffer<double>(buffer_size);

    buffer_size = 20;
    theta_time_buffer = boost::circular_buffer<double>(buffer_size);
    theta_buffer = boost::circular_buffer<float>(buffer_size);
  }

private:
  void absolute_state_callback(const geometry_msgs::msg::Pose2D & msg)
  {
    _current_count_theta = static_cast<float>(msg.theta);
    _current_x_pos = msg.x;

    // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    double now_sec = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1e6);

    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "Received absolute state: theta='%f', x_pos='%f' at time '%u'", 
    //   msg.theta, msg.x, static_cast<uint>(now_sec)
    // );

    // Add to deque
    pos_buffer.push_back(_current_x_pos);
    theta_buffer.push_back(_current_count_theta);
    time_buffer.push_back(now_sec);
    theta_time_buffer.push_back(now_sec);

    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "BUFFER: '%f', '%f', '%f', '%f' at time '%f'", 
    //   pos_buffer[0], pos_buffer[1], time_buffer[0], time_buffer[1], now_sec
    // );

    // Interpolate full state and publish
    // float pos_state = linear_interp(&pos_buffer, &time_buffer, now_sec);
    // float theta_state = linear_interp(&theta_buffer, &time_buffer, now_sec);
    float theta_dot;
    float pos_dot;
    try{
      theta_dot = average_first_difference(theta_buffer, theta_time_buffer);
      pos_dot = average_first_difference(pos_buffer, time_buffer);
    }catch(std::invalid_argument &e){
      RCLCPP_INFO(
        this->get_logger(), 
        "Not enough points to compute the average first difference."
      );
    }
   
    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "Encoder steps: '%f', Theta_dot: '%f', x_dot='%f' at time '%f'", 
    //   _current_count_theta, theta_dot, pos_dot, now_sec
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
    // Currently the data is 4000 and -4000 at theta = pi, and 0 at theta = 0. Convert to 0 and 4000 at 0, and 2000 at pi.
    if (_current_count_theta < 0){
      _current_count_theta += 8000;
    }
    // now _current_count_theta is between 0 and 4000
    float theta_radians = _current_count_theta * 2 * M_PI / 8000;
    // float theta_radians = ((2000) + static_cast<float>(_current_count_theta)) / 2000.0;
    float theta_dot_radians = theta_dot * 2 * M_PI / 8000.0;
    full_state_msg.data = {_current_x_pos, pos_dot, theta_radians, theta_dot_radians};

    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "Full state: x='%f', x_dot='%f', theta='%f', theta_dot='%f' at time '%f'", 
    //   full_state_msg.data[0], full_state_msg.data[1], full_state_msg.data[2], full_state_msg.data[3], now_sec
    // );


    full_state_publisher_->publish(full_state_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr absolute_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr full_state_publisher_;

  float _current_count_theta;
  float _current_x_pos;

  boost::circular_buffer<float> pos_buffer;
  boost::circular_buffer<float> theta_buffer;
  boost::circular_buffer<double> time_buffer;
  boost::circular_buffer<double> theta_time_buffer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FullStateEstimator>());
  rclcpp::shutdown();
  return 0;
}
