# ROS 2 SO-ARM100

Compact ROS 2 description, drivers and MoveIt configuration for the **[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)** robotic arm.

> [!NOTE]
> *Fully compatible with the mechanically-identical **SO-ARM101**.*

<table>
  <tr>
    <td style="vertical-align: top; padding-right: 16px;">
      <div style="margin-bottom: 12px;">
        <video
          src="https://github.com/user-attachments/assets/5655b956-5536-4143-9707-17cad5d1cbc8"
          controls
          muted
          loop
        ></video>
      </div>
      <div>
        <video
          src="https://github.com/user-attachments/assets/abf55009-3655-4e24-bebe-3b65d5361f03"
          controls
          muted
          loop
        ></video>
      </div>
    </td>
    <td style="vertical-align: top;">
      <video
        src="https://github.com/user-attachments/assets/36ccaca0-82dd-4206-a4dd-953867e89a20"
        controls
        muted
        loop
      ></video>
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

# 3. (Optional) MuJoCo packages
git clone https://github.com/ros-controls/mujoco_ros2_control.git

# 4. Pull the CAD submodule
git -C ros2_so_arm100 submodule update --init --recursive

# 5. Install ROS dependencies
cd ~/so_arm_ws
rosdep install --from-paths src --ignore-src -r -y

# 6. Build
colcon build --symlink-install
source install/setup.bash
```


## Quick Start

### Full Demo (RViz + controllers + MoveIt)

```bash
ros2 launch so_arm100_moveit_config demo.launch.py \
  hardware_type:=mock_components   # or :=real or :=mujoco
```

> [!TIP]
>
> - `mock_components` -> RViz-only
> - `real` -> USB, default to `/dev/LeRobotFollower`, override with `usb_port:=<device>`
> - `mujoco` -> simulation

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
