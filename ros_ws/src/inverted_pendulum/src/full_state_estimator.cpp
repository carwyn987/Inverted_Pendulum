#include <functional>
#include <memory>
#include <boost/circular_buffer.hpp>

#include "inverted_pendulum/interpolate.hpp"

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>

using std::placeholders::_1;

class FullStateEstimator : public rclcpp::Node
{
public:
  FullStateEstimator()
  : Node("full_state_estimator")
  {
    absolute_state_subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "absolute_state", 10, std::bind(&FullStateEstimator::absolute_state_callback, this, _1));

    // initialize our buffers
    uint buffer_size = 3;
    pos_buffer = boost::circular_buffer<float>(buffer_size);
    theta_buffer = boost::circular_buffer<float>(buffer_size);
  }

private:
  void absolute_state_callback(const geometry_msgs::msg::Pose2D & msg)
  {
    _current_count_theta = msg.theta;
    _current_x_pos = msg.x;

    auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    RCLCPP_INFO(
      this->get_logger(), 
      "Received absolute state: theta='%f', x_pos='%f' at time '%u'", 
      msg.theta, msg.x, static_cast<uint>(now_sec)
    );

    // Add to deque
    pos_buffer.push_back(_current_x_pos);
    theta_buffer.push_back(_current_count_theta);

    // Interpolate full state and publish
    float new_state = savitzky_golay_interpolate();
    std::cout << "New state: " << new_state << std::endl;
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr absolute_state_subscription_;

  int _current_count_theta;
  float _current_x_pos;

  boost::circular_buffer<float> pos_buffer;
  boost::circular_buffer<float> theta_buffer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FullStateEstimator>());
  rclcpp::shutdown();
  return 0;
}
