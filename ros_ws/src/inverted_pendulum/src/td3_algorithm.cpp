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
#include <cmath>

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

    // Log the input (state) to the network, as well as tthe output (output):
    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "State Copy='%0.3f %0.5f %0.3f %0.5f %0.3f %0.5f', Output='%0.4f'", 
    //   state_copy[0], state_copy[1], state_copy[2], state_copy[3], state_copy[4], state_copy[5], output[0].item<float>()
    // );


    std_msgs::msg::Float32 msg;

    if (state[4] < -2.0 || state[4] > 2.0) {
      state[4] = 0.0;
    }

    // Print the state in one line
    std::string state_str;
    for (float value : state) {
      state_str += std::to_string(value) + " ";
    }
    RCLCPP_INFO(
      this->get_logger(),
      "State: '%s'",
      state_str.c_str()
    );

    // CUSTOM SWING UP CODE:
    float angle_mod_2pi = fmod(state[5], 2 * M_PI);
    float radians_from_horiz = 0.15;
    float rl_takeover = 0.8;
    float distance_to_recenter = 0.1;

    if (angle_mod_2pi < rl_takeover || angle_mod_2pi > 2 * M_PI - rl_takeover) {
      
      // PID way
      float proportional_coefficient = 60.0;
      float integral_coefficient = 1.0;
      float derivative_coefficient = 30.0;
      // float x_change_desired_angle = 0.1;

      float desired_angle = 0.0;
      // keep centered on x:
      float distance_to_center = state[4];
      if (abs(distance_to_center) > 0.05) {
        desired_angle = distance_to_center/abs(distance_to_center) * 0.01; // distance_to_center * x_change_desired_angle;
      }

      float error = desired_angle - state[5];
      if (error < -1 * M_PI){
        error  += 2 * M_PI;
      }

      //  wash out huge error issues at start.
      if (error > M_PI || error < -1 * M_PI){
        error = 0.0;
      }

      error_integral += error;
      float error_derivative = error - lasterror;
      lasterror = error;

      msg.data = proportional_coefficient * error + integral_coefficient * error_integral + derivative_coefficient * error_derivative;

      // stop from hitting ends
      if (state[4] > 0.17 && msg.data > 0){
        msg.data = 0.0;
      } else if (state[4] < -0.17 && msg.data < 0){
        msg.data = 0.0;
      }

      // print the error values
      RCLCPP_INFO(
        this->get_logger(),
        "Error: '%.4f', Int: '%.4f', Der: '%.4f', Theta: '%.4f', Prev Theta: '%.4f', Out: '%.4f', X: '%.4f', DesAng: '%.4f'",
        error, error_integral, error_derivative, state[5], state[1], msg.data, state[4], desired_angle
      );
      
      /*
      // NN way

      // make a copy of state
      std::vector<float> state_copy = state;
      // scale up the x values (index 0,2,4) to the full range, aka [-0.17, 0.17] -> [-1,1]
      for (int i = 0; i < 6; i += 2) {
        state_copy[i] = state_copy[i] / 0.17;
      }
      // Create a vector of inputs.
      std::vector<torch::jit::IValue> inputs;
      // Turn state into a torch tensor
      torch::Tensor state_tensor = torch::from_blob(state_copy.data(), {1, 6});
      inputs.push_back(state_tensor);

      // Execute the model and turn its output into a tensor.
      at::Tensor output = module.forward(inputs).toTensor();
      std::cout << output.slice(1, 0, 5) << '\n'; // dim, start, end
      msg.data = -1.0 * output[0].item<float>();
      */

      /*
      // Energy Way (no mass)

      // Print all state values inline
      std::string state_values;
      for (float value : state) {
        state_values += std::to_string(value) + " ";
      }
      RCLCPP_INFO(
        this->get_logger(),
        "State values: '%s'",
        state_values.c_str()
      );

      // Energy Way (no mass)
      float gravity = 9.81;
      float length = 1;
      float w = (state[5] - state[1]) / 0.010; // angular velocity
      
      float w_dir = 0;
      if (abs(w) > 0.0001){
        w_dir = (w / fabs(w));
      }
      
      float epsilon = 0.0;
      float theta = fmod(state[5], 2 * M_PI) + (state[5] - state[1]); // future estimate of theta
      int theta_dir = 0;
      if (state[5] > 0 && state[5] < M_PI * 0.5) {
          theta_dir = 1;
      } else if (state[5] > M_PI * 1.5 && state[5] < M_PI * 2) {
          theta_dir = -1;
      }


      float desired_energy = gravity * length / 2.0;
      float rotational_energy = 0.5 * (1.0 / 3.0) * length * length * w * w;
      float potential_energy = gravity * (length / 2.0) * sin((M_PI / 2.0) - theta);
      bool rotating_towards_top = (theta > 0 && theta < M_PI / 2 && w < 0) || (theta <= 2 * M_PI && theta > 1.5 * M_PI && w > 0);

      // float scale_by_energy_off = 1.0 / 2.0;

      if (rotating_towards_top) {
          float total_energy = potential_energy + rotational_energy;
          if (desired_energy - total_energy > epsilon) { // we need more energy
              msg.data = 1.0 * w_dir; // * (fmin(desired_energy - total_energy, 2) * scale_by_energy_off);
          } else if (total_energy - desired_energy > epsilon) { // remove energy
              msg.data = -1.0 * w_dir; // * (fmin(total_energy - desired_energy, 2) * scale_by_energy_off);
          } else {
              msg.data = 0.0;
          }
      } else {
          // rotate to negate w
          msg.data = -1 * theta_dir;
      }
      RCLCPP_INFO(
        this->get_logger(), 
        "theta: '%.4f', w: '%.4f', des.en: '%.4f', tot.en: '%.4f', rot.en: '%.4f', pot_en: '%.4f', rot_tow_top: '%d', msg: '%.4f'",
        theta, w, desired_energy, potential_energy + rotational_energy, rotational_energy, potential_energy, rotating_towards_top, msg.data
      );
      */

    } else if (angle_mod_2pi > M_PI * (0.5 - radians_from_horiz) && angle_mod_2pi < M_PI * (0.5 + radians_from_horiz) && abs(state[4]) > distance_to_recenter) {
      // move to the center
      msg.data = -1.0 * state[4]/abs(state[4]);
    } else if (angle_mod_2pi > (1.5 - radians_from_horiz) * M_PI && angle_mod_2pi < (1.5 + radians_from_horiz) * M_PI && abs(state[4]) > distance_to_recenter){
      // move to the center
      msg.data = -1.0 * state[4]/abs(state[4]);
    } else if (angle_mod_2pi <= M_PI * 0.5) {
      msg.data = -1.0;
    } else if (angle_mod_2pi < M_PI && angle_mod_2pi >= M_PI * 0.5) {
      msg.data = 1.0;
    } else if (angle_mod_2pi > M_PI && angle_mod_2pi <= 1.5 * M_PI) {
      msg.data = -1.0;
    } else if (angle_mod_2pi >= 1.5 * M_PI && angle_mod_2pi < 2 * M_PI) {
      msg.data = 1.0;
    } else {
      msg.data = 0.0;
      RCLCPP_INFO(
        this->get_logger(), 
        "THIS SHOULDNT BE POSSIBLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
      );
    }

    
    // msg.data = output[0].item<float>();
    inference_publisher_->publish(msg);

    //  Print the current time
    // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "time '%u'",
    //   static_cast<uint>(now_sec)
    // );

    // Send to socket to enable communication
    // send_to_socket(sock, output[0].item<float>());
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr full_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr inference_publisher_;

  torch::jit::script::Module module;
  int sock;

  float error_integral = 0.0;
  float lasterror = 0.0;

  std::vector<float> last_state = {0.0, 0.0, 0.0, 0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TD3Algorithm>());
  rclcpp::shutdown();
  return 0;
}