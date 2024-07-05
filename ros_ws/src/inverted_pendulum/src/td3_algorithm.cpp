/*
  # Follow tutorial at
  https://pytorch.org/cppdocs/installing.html
  https://pytorch.org/tutorials/advanced/cpp_export.html

  ### ADD PATHS TO LIBTORCH TO .BASHRC ###
  ### Install at https://pytorch.org/
  ```
  export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/home/carwyn/Downloads/libtorch
  # Solves Error:
  # while loading shared libraries: libc10.so: cannot open shared object file: No such file or directory
  # [ros2run]: Process exited with failure 127
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/carwyn/Downloads/libtorch/lib/
  ```

  ### ADD PATH TO LIBTORCH TO CMAKE ###
  ```
  find_package(Torch REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
  ament_target_dependencies(td3_algo rclcpp std_msgs Torch) # add Torch to dependencies
  target_link_libraries(td3_algo "${TORCH_LIBRARIES}") # link torch libraries
  ```
*/

#include <torch/script.h> // One-stop header.
// #include <torch/torch.h>
// #include <iostream>
#include <functional>
#include <memory>

#include "inverted_pendulum/socket_helper.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using std::placeholders::_1;

class TD3Algorithm : public rclcpp::Node
{
public:
  TD3Algorithm()
  : Node("td3_algorithm")
  {
    full_state_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "full_state", 10, std::bind(&TD3Algorithm::full_state_callback, this, _1));
    inference_publisher_ = this->create_publisher<std_msgs::msg::Float32>("inference", 10);

    // Load the model
    try {
      // Deserialize the ScriptModule from a file using torch::jit::load().
      module = torch::jit::load("/home/carwyn/dev/Inverted_Pendulum/models/policy.pt");
    }
    catch (const c10::Error& e) {
      std::cerr << "error loading the model\n";
    }

    // sock = setup_sending_socket();
  }

private:
  void full_state_callback(const std_msgs::msg::Float32MultiArray & msg)
  {
    // double pos = msg.data[0];
    // double theta = msg.data[1];
    // double pos_dot = msg.data[2];
    // double theta_dot = msg.data[3];

    // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "Received full state at time '%u'. State=['%f', '%f', '%f', '%f']", 
    //   static_cast<uint>(now_sec), pos, theta, pos_dot, theta_dot
    // );

    last_state = msg.data; // save message data

    publish_inference(msg.data);
  }

  void publish_inference(std::vector<float> state)
  {
    // make a copy of state
    std::vector<float> state_copy = state;
    state_copy[0] *= 5.0;

    // Create a vector of inputs.
    std::vector<torch::jit::IValue> inputs;
    // Turn state into a torch tensor
    torch::Tensor state_tensor = torch::from_blob(state_copy.data(), {1, 4});
    inputs.push_back(state_tensor);

    // Execute the model and turn its output into a tensor.
    at::Tensor output = module.forward(inputs).toTensor();
    std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';

    // Given output, which is a force, compute the approximate step
    // float velocity = state[1];
    // float force = output[0].item<float>();
    // float acceleration = force / 0.1; // mass = insignificant
    // float time = 3 / 1000.0; // 1 ms
    // float difference_in_velocity = acceleration * time;
    // float new_velocity = velocity + difference_in_velocity;
    // if (new_velocity < -1.0) {
    //   new_velocity = -1.0;
    // }else if (new_velocity > 1.0) {
    //   new_velocity = 1.0;
    // }

    // Log the input (state) to the network, as well as tthe output (output):
    RCLCPP_INFO(
      this->get_logger(), 
      "State Copy='%0.2f %0.2f %0.2f %0.2f', Output='%0.2f'", 
      state_copy[0], state_copy[1], state_copy[2], state_copy[3], output[0].item<float>()
    );

    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "Vel='%0.2f', Force='%0.2f', Accel='%0.2f', New Vel='%0.2f', Last State='%0.2f %0.2f %0.2f %0.2f'", 
    //   velocity, force, acceleration, new_velocity, last_state[0], last_state[1], last_state[2], last_state[3]
    // );

    std_msgs::msg::Float32 msg;
    msg.data = output[0].item<float>();
    inference_publisher_->publish(msg);

    // Send to socket to enable communication
    // send_to_socket(sock, output[0].item<float>());
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr full_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr inference_publisher_;

  torch::jit::script::Module module;
  int sock;

  std::vector<float> last_state = {0.0, 0.0, 0.0, 0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TD3Algorithm>());
  rclcpp::shutdown();
  return 0;
}