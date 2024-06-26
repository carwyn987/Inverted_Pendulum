#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int16.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SimulatedEncoder : public rclcpp::Node
{
public:
  SimulatedEncoder()
  : Node("simulated_encoder"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int16>("encoder_pulse", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimulatedEncoder::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std_msgs::msg::Int16 msg;
    msg.data = 1;
    
    auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    RCLCPP_INFO(
      this->get_logger(), 
      "Publishing: '%d' at time '%u'", 
      msg.data, static_cast<uint>(now_sec)
    );
    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  printf("Starting up simulated encoder - inverted_pendulum package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatedEncoder>());
  rclcpp::shutdown();
  return 0;
}
