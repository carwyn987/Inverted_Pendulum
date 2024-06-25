cmake_minimum_required(VERSION 3.8)
project(inverted_pendulum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # ADDED dependency on external* package
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(encoder src/simulated_encoder.cpp)
ament_target_dependencies(encoder rclcpp std_msgs)

target_include_directories(encoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(encoder PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(state_estimator src/state_estimator.cpp)
ament_target_dependencies(state_estimator rclcpp std_msgs geometry_msgs)

# Include include directory
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(full_state_estimator src/full_state_estimator.cpp)
ament_target_dependencies(full_state_estimator rclcpp geometry_msgs)

install(TARGETS 
  encoder
  state_estimator
  full_state_estimator
  DESTINATION lib/${PROJECT_NAME})

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