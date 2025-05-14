#!/bin/bash

# Kill all ros2 launch processes for your packages
pkill -f "ros2 launch life_manager"
pkill -f "ros2 launch node_1"
pkill -f "ros2 launch node_2"

# Optionally, kill gnome-terminals running those launches
for pkg in life_manager node_1 node_2; do
    terminals=$(pgrep -af "gnome-terminal.*ros2 launch $pkg" | awk '{print $1}')
    if [ -n "$terminals" ]; then
        echo "Killing gnome-terminal(s) for $pkg: $terminals"
        kill $terminals
    fi
done