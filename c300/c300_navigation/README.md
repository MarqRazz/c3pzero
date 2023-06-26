# c300_navigation

This package contains the Nav2 parameters to be used with the Permobile c300 mobile base.
To test out this navigation package first start the robot in Gazebo and then run the included navigation launch file.

https://github.com/MarqRazz/c3pzero/assets/25058794/35301ba1-e443-44ff-b6ba-0fabae138205

# Nav2 with Gazebo simulated robot

To start the `c300` mobile base in Gazebo run the following command:
``` bash
ros2 launch c300_bringup gazebo_c300.launch.py launch_rviz:=false
```

To start Nav2 with the included Depot map run:
``` bash
ros2 launch c300_navigation navigation.launch.py
```

# Nav2 with Isaac simulated robot

To start Isaac with the `c300` mobile base in an industrial warehouse run the following command on the host PC where Isaac is installed:
``` bash
cd <path_to_workspace>/c3pzero_ws/src/c3pzero/c300/c300_description/usd
./python.sh isaac_c300.py
```

In the Docker container start the `c300` controllers to command the base and report state:
``` bash
ros2 launch c300_bringup isaac_c300.launch.py launch_rviz:=false
```

In the Docker container start Nav2 with the included Isaac Warehouse map run:
``` bash
ros2 launch c300_navigation navigation.launch.py map:=isaac_warehouse.yaml
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
