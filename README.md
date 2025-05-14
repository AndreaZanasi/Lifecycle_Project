# try_lifecycle_manager

A ROS 2 project demonstrating lifecycle management of nodes with heartbeat monitoring and automatic recovery using a LifeManager node.

## Overview

This project contains:
- **node_1** and **node_2**: ROS 2 lifecycle nodes that periodically publish heartbeat messages.
- **life_manager**: A ROS 2 node that monitors the heartbeats of managed nodes, detects failures (missed heartbeats), and automatically attempts to recover nodes by transitioning their lifecycle states.

## Features

- **Lifecycle Management**: Uses ROS 2 managed node lifecycle (configure, activate, deactivate, cleanup, shutdown).
- **Heartbeat Monitoring**: Each node publishes a heartbeat message at a configurable period.
- **Automatic Recovery**: If a node stops sending heartbeats (simulated error), the LifeManager detects this and tries to bring the node back up.
- **Parameterization**: All timing and node lists are configurable via YAML files.

## Project Structure

```
try_lifecycle_manager/
├── src/
│   ├── life_manager/
│   │   ├── params.yaml
│   │   ├── life_manager_launch.py
│   │   └── life_manager.cpp
│   ├── node_1/
│   │   ├── params.yaml
│   │   ├── launch/node_1_launch.py
│   │   └── node_1.cpp
│   └── node_2/
│       ├── params.yaml
│       ├── launch/node_2_launch.py
│       └── node_2.cpp
├── start.sh
├── set_state.sh
└── README.md
```

## How It Works

- **node_1** and **node_2** periodically publish to `/heartbeat/node_1` and `/heartbeat/node_2`.
- **life_manager** subscribes to these topics, and starts a watchdog timer for each node.
- If a heartbeat is missed (e.g., due to a simulated error in the node), the LifeManager attempts to recover the node by transitioning its lifecycle state (deactivate, activate, etc.).

## Usage

### 1. Build the Workspace

```bash
colcon build --symlink-install
source setup.bash
```

### 2. Configure Parameters

Edit the YAML files in each package's `config/params.yaml` to adjust heartbeat periods, node names, and manager settings.

Example (`src/life_manager/config/params.yaml`):

```yaml
life_manager:
  ros__parameters:
    k: 3
    nodes: ["node_1", "node_2"]
    node_config:
      node_1:
        period: 1000  # ms
      node_2:
        period: 500   # ms
```

### 3. Launch All Nodes

Use the provided script to launch all nodes in separate terminals:

```bash
start.sh
```

Or launch nodes individually:

```bash
ros2 launch life_manager life_manager_launch.py
ros2 launch node_1 node_1_launch.py
ros2 launch node_2 node_2_launch.py
```

### 4. Simulate and Observe Recovery

- `node_1` will simulate an error after a set number of heartbeats and deactivate itself.
- The LifeManager will detect the missed heartbeat and attempt to recover the node automatically.

### 5. Manually Control Node States

Use the helper script to change lifecycle states:

```bash
#to set the state of all the nodes
set_state.sh activate
#to set the state for a specific node
set_state.sh deactivate node_1
```

Or use ROS 2 CLI directly:

```bash
ros2 lifecycle set /node_1 activate
```
## Stopping All Nodes

The `stop.sh` script is provided to cleanly stop all nodes and terminals started by `start.sh`. It works by:

- Killing all `ros2 launch` processes related to `life_manager`, `node_1`, and `node_2`.

This ensures that all running nodes and their associated terminals are properly terminated, preventing orphaned processes from remaining after you stop the system.

## Customization

- **Add More Nodes**: Add node names to the `nodes` parameter in the LifeManager config and provide corresponding launch/config files.
- **Change Heartbeat Period**: Adjust the `period` parameter for each node in the YAML config.

## Requirements

- ROS 2 Foxy or newer
- Colcon build system
- `gnome-terminal` (for `start.sh` script)