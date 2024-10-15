from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

"""Launches the full game simulation with 5v5"""

team_config = {
    "Blue1": 10010,
    "Blue2": 10020,
    "Blue3": 10030,
    # "Blue4": 10040,
    # "Blue5": 10050,
    "Red1": 10110,
    "Red2": 10120,
    "Red3": 10130,
    # "Red4": 10140,
    # "Red5": 10150,
}


def generate_launch_description():
    launch_description = LaunchDescription()
    for namespace, _ in team_config.items():
        launch_description.add_entity(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nomadz_bringup"),
                            "launch",
                            "sim_robot_nodes_without_lola_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "robot_namespace": namespace,
                }.items(),
            )
        )

    return launch_description
