# c3pzero Mobile Manipulator

### To view the robots URDF in Rviz you can use the following launch file:
``` bash
ros2 launch c3pzero_description view_urdf.launch.xml
```

<img src="../doc/c3pzero_urdf.png"  width="50%" >

### To start the `c3pzero` mobile robot in Gazebo run the following command:
``` bash
ros2 launch c3pzero_bringup gazebo_c3pzero.launch.py headless:=false rviz:=true
```

### To start MoveIt to control the simulated robot run the following command:
``` bash
ros2 launch c3pzero_moveit_config demo.launch.xml mock_hardware:=true
```

or if simulating in Gazebo or Isaac:
``` bash
ros2 launch c3pzero_moveit_config demo.launch.xml use_sim_time:=true
```

### To test out the controllers in simulation you can run the following commands:

- Example arm pose
``` bash
ros2 topic pub /right_arm_jtc/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [right_arm_joint_1, right_arm_joint_2, right_arm_joint_3, right_arm_joint_4, right_arm_joint_5, right_arm_joint_6, ],
  points: [
    { positions: [-0.2, 0.85, -0.75, 0.0, 0.5, 0.0], time_from_start: { sec: 1 } },
  ]
}" -1
```
``` bash
ros2 topic pub /left_arm_jtc/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [left_arm_joint_1, left_arm_joint_2, left_arm_joint_3, left_arm_joint_4, left_arm_joint_5, left_arm_joint_6, ],
  points: [
    { positions: [-2.0, 0.85, -1.0, 0.0, 1.2, 0.0], time_from_start: { sec: 1 } },
  ]
}" -1
```

- Arm retracted pose
``` bash
ros2 topic pub /right_arm_jtc/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [right_arm_joint_1, right_arm_joint_2, right_arm_joint_3, right_arm_joint_4, right_arm_joint_5, right_arm_joint_6, ],
  points: [
    { positions: [-0.2, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 1 } },
  ]
}" -1
```
``` bash
ros2 topic pub /left_arm_jtc/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [left_arm_joint_1, left_arm_joint_2, left_arm_joint_3, left_arm_joint_4, left_arm_joint_5, left_arm_joint_6, ],
  points: [
    { positions: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 1 } },
  ]
}" -1
```

- GripperActionController (Closed position: 0.0, Open: 0.38)
``` bash
ros2 action send_goal /right_arm_gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {name: [gripper_joint], position: [0.03]}}"
```
``` bash
ros2 action send_goal /left_arm_gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {name: [gripper_joint], position: [0.03]}}"
```

### To test sending commands directly to Isaac Sim you can run the following commands:
> NOTE: sending command that are far away from the robots current pose can cause the simulation to go unstable and be thrown around in the world.

- Arm home pose
``` bash
ros2 topic pub /isaac_joint_commands sensor_msgs/JointState "{
  name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  position: [-0.2, 0.85, -0.75, 0.0, 0.5, 0.0]
}" -1
```

- Arm retracted pose
``` bash
ros2 topic pub /isaac_joint_commands sensor_msgs/JointState "{
  name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  position: [-0.2, 0.85, -0.75, 0.0, 0.5, 0.0]
}" -1
```
