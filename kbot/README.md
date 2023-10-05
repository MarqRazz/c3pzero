# Creating a URDF
To create URDF for Isaac Sim to import run the following command from the directory `ros2_kortex/kortex_description/robots`:
ros2 run xacro xacro gen3.xacro > gen3.urdf


# View robot in Rviz
To view the robot in Rviz run the following command:
ros2 launch kortex_description view_robot.launch.py




2023-03-23 03:28:19 [277,497ms] [Warning] [omni.physx.plugin] The rigid body at /gen3/end_effector_link has a possibly invalid inertia tensor of {1.0, 1.0, 1.0} and a negative mass, small sphere approximated inertia was used. Either specify correct values in the mass properties, or add collider(s) to any shape(s) that you wish to automatically compute mass properties for. If you do not want the objects to collide, add colliders regardless then disable the 'enable collision' property.
2023-03-23 03:28:19 [277,498ms] [Warning] [omni.physx.plugin] The rigid body at /gen3/tool_frame has a possibly invalid inertia tensor of {1.0, 1.0, 1.0} and a negative mass, small sphere approximated inertia was used. Either specify correct values in the mass properties, or add collider(s) to any shape(s) that you wish to automatically compute mass properties for. If you do not want the objects to collide, add colliders regardless then disable the 'enable collision' property.
2023-03-23 03:28:19 [277,498ms] [Warning] [omni.physx.plugin] The rigid body at /gen3/world has a possibly invalid inertia tensor of {1.0, 1.0, 1.0} and a negative mass, small sphere approximated inertia was used. Either specify correct values in the mass properties, or add collider(s) to any shape(s) that you wish to automatically compute mass properties for. If you do not want the objects to collide, add colliders regardless then disable the 'enable collision' property.

Error loading the robot:
2023-03-23 03:29:27 [344,877ms] [Warning] [omni.usd] Coding Error: in Get at line 339 of ../../../source/extensions/omni.hydra.scene_delegate/plugins/omni.hydra.scene_delegate.plugin/OmniHydraDelegate.cpp -- Failed verification: ' cacheItem ' -- </gen3/spherical_wrist_2_link/visuals/mesh_0> (wgs84Location)

Error when executing some trajectories:
2023-03-26 20:25:26 [65,650ms] [Error] [omni.physx.plugin] PhysX error: PxArticulationJointReducedCoordinate::setDriveTarget() only supports target angle in range [-2Pi, 2Pi] for joints of type PxArticulationJointType::eREVOLUTE, FILE /buildAgent/work/16dcef52b68a730f/source/physx/src/NpArticulationJointReducedCoordinate.cpp, LINE 299
^C2023-03-26 20:25:26 [65,708ms] [Warning] [omni.kit.notification_manager.manager] PhysX error: PxArticulationJointReducedCoordinate::setDriveTarget() only supports target angle in range [-2Pi, 2Pi] for joints of type PxArticulationJointType::eREVOLUTE


# Setup assistant warnings:
When creating the manipulator chain it prints:
[moveit_setup_assistant-1] Warning: Group 'manipulator' is empty.
[moveit_setup_assistant-1]          at line 245 in ./src/model.cpp
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846352.649289902]: Loading robot model 'gen3'...
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846352.649307000]: No root/virtual joint specified in SRDF. Assuming fixed joint
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846352.730881482]: Group 'manipulator' must have at least one valid joint
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846352.730905008]: Failed to add group 'manipulator'

same with the gripper:
[moveit_setup_assistant-1] Warning: Group 'gripper' is empty.
[moveit_setup_assistant-1]          at line 245 in ./src/model.cpp
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846703.525177444]: Loading robot model 'gen3'...
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846703.525195566]: No root/virtual joint specified in SRDF. Assuming fixed joint
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846703.622663966]: Group 'gripper' must have at least one valid joint
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846703.622684222]: Failed to add group 'gripper'

It would be nice if selecting the Kinematic chain alreay had the robot links expanded!

Every time you switch to the 'Planning Groups` tab it prints:
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846528.501608700]: Loading robot model 'gen3'...
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846528.501632328]: No root/virtual joint specified in SRDF. Assuming fixed joint


When defining the End Effectors:
- What is the `Parent Link (usually part of the arm)`?
- What is the `Parent Group (optional)`? Adding this makes it throw and error:
The specified parent group 'manipulator' must contain the specified parent link 'base_link'.
- Why would I set them?
- Adding it prints the fillowing warning:
[moveit_setup_assistant-1] [moveit_robot_model.robot_model 1679846928.510935526]: Could not identify parent group for end-effector 'gripper'

When creating the gripper moveit_controllers.yaml:
- it creates a controller as `FollowJointTrajectory` not `GripperCommand` with a topic `gripper_cmd`

When Generating Configuration Files:
- You have to use the create folder button to name your package
- What is the bottom text box and why can't I type in it? -> its not a text box its a loading bar!
- The pop-up when unselecting items to be generated is super annoying!
- several configs are in the list twice
