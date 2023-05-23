# c300_navigation

This package contains the Nav2 parameters to be used with the Permobile c300 mobile base.
To test out this navigation package first start the robot in Gazebo and then run the included navigation launch file.

## To start the `c300` mobile base in Gazebo run the following command:
``` bash
ros2 launch c300_bringup c300_sim.launch.py launch_rviz:=false
```

## To start navigation run the following command:
``` bash
ros2 launch c300_navigation navigation.launch.py
```
