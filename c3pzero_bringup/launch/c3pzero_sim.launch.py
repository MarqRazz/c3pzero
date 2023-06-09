# -*- coding: utf-8 -*-
# Author: Marq Rasmussen

import shlex
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="True",
            description="Use Ignition Gazebo for simulation",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="c3pzero_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="c3pzero_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="c3pzero_kinova_gen3.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="c3pzero",
            description="Robot name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "diff_drive_controller",
            default_value="diff_drive_base_controller",
            description="Diff drive base controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "jtc_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_pos_controller",
            default_value="streaming_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_hand_controller",
            default_value="robotiq_gripper_controller",
            description="Robot hand controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="True", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Initialize Arguments
    sim_ignition = LaunchConfiguration("sim_ignition")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    diff_drive_controller = LaunchConfiguration("diff_drive_controller")
    robot_traj_controller = LaunchConfiguration("jtc_controller")
    robot_pos_controller = LaunchConfiguration("robot_pos_controller")
    robot_hand_controller = LaunchConfiguration("robot_hand_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "rviz", "bringup_config.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_content,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_traj_controller, "-c", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[diff_drive_controller, "-c", "/controller_manager"],
    )

    # streaming controller
    # robot_pos_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[robot_pos_controller, "-c", "/controller_manager"],
    # )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_hand_controller, "-c", "/controller_manager"],
    )

    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.3",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        condition=IfCondition(sim_ignition),
    )

    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        # TODO (marqrazz): fix the hardcoded path to the gazebo world
        launch_arguments={
            "gz_args": " -r -v 3 /root/c3pzero_ws/src/c3pzero/c300/c300_bringup/worlds/depot.sdf"
        }.items(),
        condition=IfCondition(sim_ignition),
    )

    # Bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/segmentation/colored_map@sensor_msgs/msg/Image[ignition.msgs.Image",
            #    '/segmentation/labels_map@sensor_msgs/msg/Image@ignition.msgs.Image',
            "/segmentation/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        robot_traj_controller_spawner,
        # robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        ignition_launch_description,
        ignition_spawn_entity,
        gazebo_bridge,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
