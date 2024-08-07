#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iostream>
#include <thread>

#include "inverted_pendulum/socket_helper.hpp"
#include "inverted_pendulum/arduino_comm.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>

// using namespace std::chrono_literals;

using std::placeholders::_1;

class CommunicationAdapter : public rclcpp::Node
{
public:
  CommunicationAdapter()
  : Node("simulated_encoder"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int16>("encoder_pulse", 10);
    stepper_publisher_ = this->create_publisher<std_msgs::msg::Float32>("stepper_moved", 10);

    inference_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "inference", 10, std::bind(&CommunicationAdapter::forward_stepper_pulse, this, _1)
    );
    
    // Set up serial connection
    const char* portname = "/dev/ttyUSB0"; // Adjust this to your port
    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portname << ": " << strerror(errno) << std::endl;
    }

    if (set_serial_attributes(fd, B115200, 0) < 0) {
        close(fd);
    }

    // Create a separate thread for the comm loop
    comm_thread_ = std::thread([this] {
      while (true) {
        comm_loop();
        // delay for 100 microseconds
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });

  }

private:

  void forward_stepper_pulse(const std_msgs::msg::Float32 & msg){

    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "Stepper pulse received and being forwarded of: '%f'", 
    //   msg.data
    // );
    // send_to_socket(sock, static_cast<float>(msg.data));

    send_floats_to_serial(fd, msg.data);
  }

  void comm_loop(){
    // float value = read_floats_from_serial(fd);
    float value = 0.0;

    SerialResult result = read_from_serial(fd);

    if (result.type == X_POSITION) {
      value = result.value;
      send_stepper_move(value);

      auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      RCLCPP_INFO(
        this->get_logger(), 
        "time '%u'",
        static_cast<uint>(now_sec)
      );

      // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      // RCLCPP_INFO(
      //   this->get_logger(), 
      //   "Got stepper reading : '%f' at time '%u'", 
      //   value, static_cast<uint>(now_sec)
      // );

    } else if (result.type == THETA) {
      value = result.value;
      send_encoder_data(value);
    }
  }

  void send_encoder_data(float data)
  {
    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int>(data);
    publisher_->publish(msg);
  }

  void send_stepper_move(float data)
  {
    std_msgs::msg::Float32 msg;
    msg.data = data;
    stepper_publisher_->publish(msg);
  }

  std::thread comm_thread_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr stepper_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr inference_subscriber_;
  size_t count_;

  int sock;
  int fd;
};

int main(int argc, char * argv[])
{
  printf("Starting up simulated encoder - inverted_pendulum package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommunicationAdapter>());
  rclcpp::shutdown();
  return 0;
}
