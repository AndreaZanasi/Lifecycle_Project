#!/bin/bash

# Paths to your ROS packages and their launch files
PACKAGE_NAME=("life_manager" "node_1" "node_2")
LAUNCH_FILES=("life_manager_launch.py" "node_1_launch.py" "node_2_launch.py")

# Open each launch file in a new terminal
for i in "${!LAUNCH_FILES[@]}"; do
    gnome-terminal -- bash -c "source install/setup.bash; ros2 launch ${PACKAGE_NAME[$i]} ${LAUNCH_FILES[$i]}; exec bash"
done