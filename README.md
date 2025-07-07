# ROS 2 SO-ARM100

Compact ROS 2 description, drivers and MoveIt configuration for the **[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)** robotic arm.

*Fully compatible with the mechanically-identical **SO-ARM101**.*

<table>
  <tr>
    <td>
      <video src="https://github.com/user-attachments/assets/5655b956-5536-4143-9707-17cad5d1cbc8"></video>
    </td>
    <td>
      <video src="https://github.com/user-attachments/assets/36ccaca0-82dd-4206-a4dd-953867e89a20"></video>
    </td>
  </tr>
</table>

## Installation

```bash
# 1. Create a workspace
mkdir -p ~/so_arm_ws/src
cd ~/so_arm_ws/src

# 2. Clone core packages
git clone https://github.com/JafarAbdi/ros2_so_arm100.git
git clone https://github.com/JafarAbdi/feetech_ros2_driver.git

# 3. Pull the CAD sub-module
git -C ros2_so_arm100 submodule update --init --recursive

# 4. Install ROS dependencies
cd ~/so_arm_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --symlink-install
source install/setup.bash
```


## Quick Start

> **hardware_type:** `mock_components` (RViz-only) | `real` (USB, default `/dev/LeRobotFollower`, override with `usb_port:=<device>`)


### Full Demo (RViz + controllers + MoveIt)

```bash
ros2 launch so_arm100_moveit_config demo.launch.py \
  hardware_type:=mock_components   # or :=real
````

### Bring-up Only (no RViz)

| Purpose               | Command                                                                                          |
| --------------------- | ------------------------------------------------------------------------------------------------ |
| MoveIt server         | `ros2 launch so_arm100_moveit_config move_group.launch.py`                                       |
| Low-level controllers | `ros2 launch so_arm100_description controllers_bringup.launch.py hardware_type:=mock_components` |

### Visualisation Shortcuts

| View                    | Command                                                                                     |
| ----------------------- | ------------------------------------------------------------------------------------------- |
| Robot model in RViz     | `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share so_arm100_description)/rviz/config.rviz` |
| RViz with MoveIt plugin | `ros2 launch so_arm100_moveit_config moveit_rviz.launch.py`                                 |

### Interact & Test

| Tool                     | Command                                                                                                                      |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------------------- |
| Joint trajectory GUI     | `ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller`                                                   |
| MoveIt Setup Assistant\* | `ros2 run moveit_setup_assistant moveit_setup_assistant --config_pkg ~/so_arm_ws/src/ros2_so_arm100/so_arm100_moveit_config` |

\*Use the assistant to tweak or regenerate MoveIt configs.
