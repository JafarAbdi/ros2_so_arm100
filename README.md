# ROS2 SO-ARM100

Description: ROS 2 description package for https://github.com/TheRobotStudio/SO-ARM100

https://github.com/user-attachments/assets/5655b956-5536-4143-9707-17cad5d1cbc8


https://github.com/user-attachments/assets/36ccaca0-82dd-4206-a4dd-953867e89a20


## Usage

To launch the demo run:

```bash
ros2 launch so_arm100_moveit_config demo.launch.py hardware_type:=mock_components # hardware_type:=real for running with hardware
```

To launch the controllers run:

```bash
ros2 launch so_arm100_description controllers_bringup.launch.py hardware_type:=mock_components # hardware_type:=real for running with hardware
```

To launch rviz run:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share so_arm100_description)/rviz/config.rviz
```

To launch rviz with moveit's Motion Planning plugin:

```bash
ros2 launch so_arm100_moveit_config moveit_rviz.launch.py
```

To launch move_group node (without rviz):
```bash
ros2 launch so_arm100_moveit_config move_group.launch.py
```

To test the joint_trajectory_controller run:

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

## MoveIt Setup Assistant

To regenerate/modify the config files using moveit_setup_assistant

```
ros2 run moveit_setup_assistant moveit_setup_assistant --config_pkg ~/PATH/ros2_so_arm100/so_arm100_moveit_config
```
