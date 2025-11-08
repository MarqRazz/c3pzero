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
    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="false", description="Launch RViz?")
    )

    # Initialize Arguments
    launch_rviz = LaunchConfiguration("rviz")
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("c3pzero_bringup"),
            "config",
            "c3pzero_isaac_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("c3pzero_bringup"), "rviz", "c3pzero.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("c3pzero_description"), "urdf", "c3pzero.urdf.xacro"]
            ),
            " ",
            "sim_isaac:=true",
            " ",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "use_sim_time": True,
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

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": True},
            robot_controllers,
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_drive_base_controller/cmd_vel", "/cmd_vel_nav"),
            ("/diff_drive_base_controller/odom", "/odom"),
        ],
        output="both",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "diff_drive_base_controller",
            "right_arm_jtc",
            "left_arm_jtc",
            "right_arm_gripper_controller",
            "left_arm_gripper_controller",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="both",
    )

    # Delay rviz start after `controller_spawner`
    delay_rviz_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner,
            on_exit=[rviz_node],
        )
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
        robot_state_publisher_node,
        delay_rviz_after_controller_spawner,
        control_node,
        controller_spawner,
        # point_cloud_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
