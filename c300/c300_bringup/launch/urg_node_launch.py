# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory("c300_bringup")
    launch_description = LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_interface",
                default_value="ethernet",
                description="sensor_interface: supported: serial, ethernet",
            )
        ]
    )

    def expand_param_file_name(context):
        param_file = os.path.join(
            config_dir,
            "config",
            "urg_node_" + context.launch_configurations["sensor_interface"] + ".yaml",
        )
        if os.path.exists(param_file):
            return [SetLaunchConfiguration("param", param_file)]

    param_file_path = OpaqueFunction(function=expand_param_file_name)
    launch_description.add_action(param_file_path)

    hokuyo_node = Node(
        package="urg_node",
        executable="urg_node_driver",
        output="screen",
        name="urg_node",
        parameters=[LaunchConfiguration("param")],
    )

    launch_description.add_action(hokuyo_node)
    return launch_description
