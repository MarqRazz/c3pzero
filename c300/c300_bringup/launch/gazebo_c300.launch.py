# -*- coding: utf-8 -*-
# Author: Marq Rasmussen

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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
from ament_index_python.packages import get_package_share_directory


def launch_gz(context: LaunchContext):
    gz_world_file = get_package_share_directory("c300_bringup") + "/worlds/depot.sdf"
    # -r is to run the simulation on start
    # -v is the verbose level
    #  0: No output, 1: Error, 2: Error and warning, 3: Error, warning, and info, 4: Error, warning, info, and debug.
    sim_options = "-r -v 3"
    if LaunchConfiguration("headless").perform(context) == "true":
        sim_options += " -s"  # -s is to only run the server (headless mode).
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [f"{sim_options} {gz_world_file}"])],
    )
    return [gz_launch_description]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="false", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless", default_value="true", description="Launch Gazebo headless?"
        )
    )

    # Initialize Arguments
    launch_rviz = LaunchConfiguration("rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("c300_bringup"), "rviz", "c300.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("c300_description"), "urdf", "c300.urdf.xacro"]
            ),
            " ",
            "sim_gazebo:=true",
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
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "diff_drive_base_controller",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `controller_spawner`
    delay_rviz_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner,
            on_exit=[rviz_node],
        )
    )

    gazebo_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "c300",
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
    )

    # Bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/gt_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher_node,
        controller_spawner,
        delay_rviz_after_controller_spawner,
        OpaqueFunction(function=launch_gz),
        gazebo_spawn_entity,
        gazebo_bridge,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
