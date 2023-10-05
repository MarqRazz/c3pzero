# c300 Mobile Base

## To visualize this robot's URDF run:
``` bash
ros2 launch c300_description view_base_urdf.launch.py
```

## To start the `c300` mobile base in Gazebo run the following command:
``` bash
ros2 launch c300_bringup gazebo_c300.launch.py
```

## To teleoperate the robot with a Logitech F710 joystick run:
``` bash
ros2 launch c300_driver teleop.launch.py
```
> NOTE: in simulation the `cmd_vel` topic is on `/diff_drive_base_controller/cmd_vel_unstamped`
