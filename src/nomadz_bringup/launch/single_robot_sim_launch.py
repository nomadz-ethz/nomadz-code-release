from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.descriptions import executable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

"""Launches the simulation of a single robot."""


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_namespace_launch_arg = DeclareLaunchArgument(
        "robot_namespace",
        description="Name which should be given to the namespace of a single robot",
        default_value="",
    )

    world_file = LaunchConfiguration("world_file", default="nao_robocup.wbt")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nomadz_bringup"),
                    "launch",
                    "sim_robot_nodes_launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_namespace": robot_namespace,
        }.items(),
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nomadz_webots_controller"),
                    "launch",
                    "supervisor_launch.py",
                ]
            )
        ),
        launch_arguments={
            "world_file": world_file,
        }.items(),
    )

    return LaunchDescription(
        [
            robot_namespace_launch_arg,
            robot,
            simulation,
        ]
    )
