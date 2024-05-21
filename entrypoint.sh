#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Run your command
ros2 launch demon lower_tk_ls.yaml

# Keep the container alive by starting a shell
exec /bin/bash
