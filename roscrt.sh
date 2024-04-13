mkdir -p ros2_ws/src
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd src
ros2 pkg create --build-type ament_python --license Apache-2.0 demon
colcon build --packages-select demon

