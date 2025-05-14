#!/bin/bash

# Usage: ./set_state.sh <state> [node1 node2 ...]
# Example: ./set_state.sh activate node1 node2

STATE="$1"
shift

if [ -z "$STATE" ]; then
    echo "Usage: $0 <state> [node1 node2 ...]"
    exit 1
fi

# Function to get all running lifecycle nodes
get_running_nodes() {
    ros2 node list | sed 's|^/||'
}

# If no nodes specified, get all running nodes
if [ "$#" -eq 0 ]; then
    NODES=$(get_running_nodes)
else
    NODES="$@"
fi

for NODE in $NODES; do
    echo "Changing state of $NODE to $STATE"
    ros2 lifecycle set "$NODE" "$STATE"
done