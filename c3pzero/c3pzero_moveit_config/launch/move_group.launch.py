# -*- coding: utf-8 -*-
# Author: Marq Rasmussen

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )

    # Initialize Arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("sim")

    # This launch file assumes the robot is already running so we don't need to
    # pass any special arguments to the URDF.xacro like sim_ignition or sim_isaac
    # because the controllers should already be running.
    description_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "false",
    }

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="c3pzero_moveit_config")
        .robot_description(mappings=description_arguments)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        arguments=[
            "--ros-args",
            "--log-level",
            "fatal",
        ],  # MoveIt is spamming the log because of unknown '*_mimic' joints
        condition=IfCondition(launch_rviz),
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("c3pzero_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(declared_arguments + [move_group_node, rviz_node])
