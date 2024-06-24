```
mkdir -p ros_ws/src/
cd ros_ws/src/
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node inverted_pendulum
pip install atkin_pkg
colcon build
source install/setup.sh
```
Naming convention for packages is lowercase, underscores. No uppercase.


Publisher Subscriber Tutorial: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
```
colcon build --packages-select inverted_pendulum
ros2 run <package> <node>
ros2 launch src/inverted_pendulum/launch/launch.xml
```