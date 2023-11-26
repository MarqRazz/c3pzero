# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def launch_setup(context, *args, **kwargs):
    # Configure some helper variables and paths
    pkg_c300_navigation = get_package_share_directory("c300_navigation")
    pkg_driver = get_package_share_directory("c300_driver")
    pkg_robot_description = get_package_share_directory("c300_description")
    pkg_bringup = get_package_share_directory("c300_bringup")

    # Hardware LIDAR
    lidar_launch_py = PythonLaunchDescriptionSource(
        [
            pkg_bringup,
            "/launch/urg_node_launch.py",
        ]
    )
    lidar_launch = IncludeLaunchDescription(
        lidar_launch_py,
    )

    # Hardware Driver
    driver_launch_py = PythonLaunchDescriptionSource(
        [
            pkg_driver,
            "/launch/driver.launch.py",
        ]
    )
    driver_launch = IncludeLaunchDescription(
        driver_launch_py,
    )

    # Parse xacro and publish robot state
    robot_description_path = os.path.join(
        pkg_robot_description, "urdf", "c300_base.urdf"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="c300_robot_state_publisher",
        output="both",
        parameters=[robot_description, {"publish_frequency", "50"}],
    )

    # Bringup Navigation2
    nav2_launch_py = PythonLaunchDescriptionSource(
        [
            pkg_c300_navigation,
            "/launch/navigation.launch.py",
        ]
    )
    nav2_launch = IncludeLaunchDescription(
        nav2_launch_py,
    )

    nodes_and_launches = [
        robot_state_publisher_node,
        lidar_launch,
        driver_launch,
        TimerAction(period=5.0, actions=[nav2_launch]),
    ]

    return nodes_and_launches


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )
