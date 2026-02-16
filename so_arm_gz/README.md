# so_arm_gz

ROS 2 Gazebo simulation package for SO-ARM100 and SO-ARM101 robotic arms.

## Overview

This package provides a launch file to simulate SO-ARM robotic arms in Gazebo using ros2_control. It supports both SO-ARM100 and SO-ARM101 arm variants with full controller integration and RViz visualization.

## Features

- URDF-based robot spawning in Gazebo
- ros2_control integration with Gazebo
- Support for both SO-ARM100 and SO-ARM101
- Automatic controller spawning
- RViz visualization
- Configurable robot pose and namespace

## Launch File

### so_arm_gz_bringup.launch.py

This launch file starts a complete Gazebo simulation environment with the following components:

**Started Nodes:**
- `gazebo` - Gazebo simulator (with Bullet physics engine)
- `robot_state_publisher` - Publishes robot state from URDF
- `joint_state_broadcaster` - Broadcasts joint states
- `forward_position_controller` (or custom) - Joint position controller
- `rviz2` - Visualization (optional, waits for joint_state_broadcaster)
- `ros_gz_bridge` - Bridges /clock topic from Gazebo to ROS

## Usage

### Basic Usage

Launch with default settings (SO-ARM101):
```bash
ros2 launch so_arm_gz so_arm_gz_bringup.launch.py
```

Launch with SO-ARM100:
```bash
ros2 launch so_arm_gz so_arm_gz_bringup.launch.py arm_id:=so_arm100
```

### Headless Mode (no GUIs)

```bash
ros2 launch so_arm_gz so_arm_gz_bringup.launch.py \
  gazebo_gui:=false \
  launch_rviz:=false
```

### Multi-Robot Setup

```bash
ros2 launch so_arm_gz so_arm_gz_bringup.launch.py \
  prefix:=robot1_ \
  use_namespace:=true \
  namespace:=robot1
```

### Custom Spawn Position

```bash
ros2 launch so_arm_gz so_arm_gz_bringup.launch.py \
  x:=1.0 \
  y:=2.0 \
  z:=0.5 \
  yaw:=1.57
```

## Launch Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `arm_id` | string | `so_arm101` | Arm variant: `so_arm100` or `so_arm101` |
| `controllers_file` | string | _(auto)_ | Path to ros2_controllers YAML. Auto-resolves to `{arm_id}_description/control/ros2_controllers.yaml` |
| `description_file` | string | _(auto)_ | Path to URDF/XACRO file. Auto-resolves to `{arm_id}_description/urdf/{arm_id}.urdf.xacro` |
| `rviz_config_file` | string | _(auto)_ | Path to RViz config. Auto-resolves to `{arm_id}_description/rviz/config.rviz` |
| `prefix` | string | `""` | Joint name prefix for multi-robot setups |
| `use_namespace` | bool | `false` | Whether to namespace the controller stack |
| `namespace` | string | `""` | Controller manager namespace |
| `activate_joint_controller` | bool | `true` | Start the joint controller immediately |
| `initial_joint_controller` | string | `forward_position_controller` | Controller to start (see available controllers below) |
| `launch_rviz` | bool | `true` | Launch RViz visualization |
| `gazebo_gui` | bool | `true` | Show Gazebo GUI |
| `x` | float | `0.0` | Robot spawn X position (meters) |
| `y` | float | `-0.488` | Robot spawn Y position (meters) |
| `z` | float | `0.845` | Robot spawn Z position (meters) |
| `roll` | float | `0.0` | Robot spawn roll orientation (radians) |
| `pitch` | float | `0.0` | Robot spawn pitch orientation (radians) |
| `yaw` | float | `-3.141` | Robot spawn yaw orientation (radians) |

## Available Controllers

The following controllers are defined in the ros2_controllers.yaml files:

- `joint_state_broadcaster` - Always started, publishes joint states
- `forward_position_controller` - Joint group position controller (default)
- `joint_trajectory_controller` - Trajectory controller for motion planning
- `gripper_controller` - Parallel gripper action controller

### Switching Controllers

After launch, you can switch between controllers:

```bash
# Stop current controller
ros2 control switch_controllers --deactivate forward_position_controller

# Start trajectory controller
ros2 control switch_controllers --activate joint_trajectory_controller
```
