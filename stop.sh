#!/bin/bash

# Stop gnome-terminal windows running your ROS 2 launch files started by start.sh

LAUNCH_FILES=("life_manager_launch.py" "node_1_launch.py" "node_2_launch.py")

for LAUNCH in "${LAUNCH_FILES[@]}"; do
    # Find gnome-terminal processes running the specific launch file
    pids=$(pgrep -f "gnome-terminal.*$LAUNCH")
    if [ -z "$pids" ]; then
        echo "No gnome-terminal found running: $LAUNCH"
    else
        echo "Killing gnome-terminal(s) for: $LAUNCH (PIDs: $pids)"
        kill $pids
    fi
done