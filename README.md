# ROS2 SO-ARM100

Description: ROS 2 description package for https://github.com/TheRobotStudio/SO-ARM100

## Usage

To launch the controllers run:

```bash
ros2 launch so_arm100_description controllers_bringup.launch.py
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
