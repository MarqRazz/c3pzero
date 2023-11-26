# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="c300_driver",
                executable="twist2roboclaw",
                name="driver",
                remappings={
                    ("c3pzero/cmd_vel", "/cmd_vel"),
                    ("/c3pzero/odometry", "/odom"),
                },
            ),
        ]
    )
