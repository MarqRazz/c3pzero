# User Guide

These instructions assume you have followed the [Installation](doc/installation.md) guide and have a terminal running inside the Docker container.

## To start the `c300` mobile base in Gazebo run the following command:
``` bash
ros2 launch c300_bringup c300_sim.launch.py
```

# To teleoperate the robot with a Logitech F710 joystick run:
``` bash
ros2 launch c300_driver teleop.launch.py
```
> NOTE: in simulation the `cmd_vel` topic is on `/diff_drive_base_controller/cmd_vel_unstamped`
