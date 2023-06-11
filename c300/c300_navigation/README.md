# c300_navigation

This package contains the Nav2 parameters to be used with the Permobile c300 mobile base.
To test out this navigation package first start the robot in Gazebo and then run the included navigation launch file.

## To start the `c300` mobile base in Gazebo run the following command:
``` bash
ros2 launch c300_bringup gazebo_c300.launch.py launch_rviz:=false
```

## To start Nav2:

To start the robot with the included Depot map run:
``` bash
ros2 launch c300_navigation navigation.launch.py
```

## Initialize the location of the robot

Currently the [nav2_parameters.yaml](https://github.com/MarqRazz/c3pzero/blob/main/c300/c300_navigation/params/nav2_params.yaml) is setup to automatically set the initial pose of the robot for simulation environments.
If running on hardware change `set_initial_pose: false` under the `amcl` parameters and use the Rviz tool to set the initial pose on startup.

You can also set the initial pose through cli with the following command:
```bash
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, } }'

```

## Running SLAM

To create a new map with SLAM Toolbox start the robot and then run:
``` bash
ros2 launch c300_navigation navigation.launch.py slam:=True
```

After you have created a new map you can save it with the following command:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/c3pzero_ws/<name of the new map>
```
