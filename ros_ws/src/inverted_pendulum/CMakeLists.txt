cmake_minimum_required(VERSION 3.8)
project(inverted_pendulum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(Torch REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # ADDED dependency on external* package
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# PYTORCH
# export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/home/carwyn/Downloads/libtorch
# set(CMAKE_PREFIX_PATH ./libtorch)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(td3_algo src/td3_algorithm.cpp)
ament_target_dependencies(td3_algo rclcpp std_msgs Torch)

# add_executable(encoder src/simulated_encoder.cpp)
add_executable(adapter src/adapter.cpp)
ament_target_dependencies(adapter rclcpp std_msgs)

# target_include_directories(adapter PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(adapter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(state_estimator src/state_estimator.cpp)
ament_target_dependencies(state_estimator rclcpp std_msgs geometry_msgs)

# Include include directory
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(full_state_estimator src/full_state_estimator.cpp)
ament_target_dependencies(full_state_estimator rclcpp geometry_msgs)

install(TARGETS 
  adapter
  state_estimator
  full_state_estimator
  td3_algo
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(td3_algo "${TORCH_LIBRARIES}") # link torch libraries


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
