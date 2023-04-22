# -*- coding: utf-8 -*-
# Copyright 2021 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="c300_description"
    ).find("c300_description")
    default_model_path = os.path.join(pkg_share, "urdf/c300_base.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/view_urdf.rviz")

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
