# -*- coding: utf-8 -*-
# Author: Marq Rasmussen

import shlex
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node
import launch_ros.descriptions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_isaac",
            default_value="True",
            description="Use Nvidia Isaac for simulation",
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
            "controllers_file",
            default_value="c3pzero_isaac_controllers.yaml",
            description="YAML file with the controllers configuration.",
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
    sim_isaac = LaunchConfiguration("sim_isaac")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    diff_drive_controller = LaunchConfiguration("diff_drive_controller")
    robot_traj_controller = LaunchConfiguration("jtc_controller")
    robot_hand_controller = LaunchConfiguration("robot_hand_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

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
            " ",
            "sim_isaac:=",
            sim_isaac,
            " ",
            "sim_ignition:=False",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_controllers,
        ],
        remappings=[
            ("/diff_drive_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/diff_drive_base_controller/odom", "/odom"),
        ],
        output="both",
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

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_hand_controller, "-c", "/controller_manager"],
    )

    # launch point cloud plugin through rclcpp_components container
    # see https://github.com/ros-perception/image_pipeline/blob/humble/depth_image_proc/launch/point_cloud_xyz.launch.py
    point_cloud_node = ComposableNodeContainer(
        name="container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # Driver itself
            launch_ros.descriptions.ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="point_cloud_xyz_node",
                remappings=[
                    ("rgb/image_rect_color", "/wrist_mounted_camera/color/image_raw"),
                    ("rgb/camera_info", "/wrist_mounted_camera/color/camera_info"),
                    (
                        "depth_registered/image_rect",
                        "/wrist_mounted_camera/depth/image_rect_raw",
                    ),
                    ("points", "/wrist_mounted_camera/depth/color/points"),
                ],
            ),
        ],
        output="screen",
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        robot_traj_controller_spawner,
        robot_hand_controller_spawner,
        point_cloud_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
