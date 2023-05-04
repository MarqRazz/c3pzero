# URDF for c3pzero
differential drive robot (permobile c300) with Kinova Gen3 arm attached.

## Launch Commands
```
ros2 launch c3pzero_description display.launch.py
ros2 launch c3pzero_bringup c3pzero_sim.launch.py
```

# home pose
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [gen3_joint_1, gen3_joint_2, gen3_joint_3, gen3_joint_4, gen3_joint_5, gen3_joint_6, gen3_joint_7],
  points: [
    { positions: [0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57], time_from_start: { sec: 2 } },
  ]
}" -1

# retract pose
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [gen3_joint_1, gen3_joint_2, gen3_joint_3, gen3_joint_4, gen3_joint_5, gen3_joint_6, gen3_joint_7],
  points: [
    { positions: [0.0, -0.35, 3.14, -2.54, 0.0, -0.87, 1.57], time_from_start: { sec: 2 } },
  ]
}" -1

# test GripperActionController (Open posiiton: 0, Closed: 0.8)
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0}}
