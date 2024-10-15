from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.descriptions import executable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

"""Launches the simulation of a single robot."""


def generate_launch_description():
    world_file = LaunchConfiguration("world_file", default="nao_robocup.wbt")

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
            simulation,
        ]
    )
