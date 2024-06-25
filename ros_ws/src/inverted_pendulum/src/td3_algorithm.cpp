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
  }

private:
  void full_state_callback(const std_msgs::msg::Float32MultiArray & msg)
  {
    double pos = msg.data[0];
    double theta = msg.data[1];
    double pos_dot = msg.data[2];
    double theta_dot = msg.data[3];

    auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    RCLCPP_INFO(
      this->get_logger(), 
      "Received full state at time '%u'. State=['%f', '%f', '%f', '%f']", 
      static_cast<uint>(now_sec), pos, theta, pos_dot, theta_dot
    );

    publish_inference(msg.data);
  }

  void publish_inference(std::vector<float> state)
  {
    // Create a vector of inputs.
    std::vector<torch::jit::IValue> inputs;
    // Turn state into a torch tensor
    torch::Tensor state_tensor = torch::from_blob(state.data(), {1, 4});
    inputs.push_back(state_tensor);

    // Execute the model and turn its output into a tensor.
    at::Tensor output = module.forward(inputs).toTensor();
    std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';

    RCLCPP_INFO(
      this->get_logger(), 
      "PREDICTION: '%f'", 
      output[0].item<float>()
    );

    std_msgs::msg::Float32 msg;
    msg.data = output[0].item<float>();
    inference_publisher_->publish(msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr full_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr inference_publisher_;

  torch::jit::script::Module module;
};

int main(int argc, char * argv[])
{  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TD3Algorithm>());
  rclcpp::shutdown();
  return 0;
}