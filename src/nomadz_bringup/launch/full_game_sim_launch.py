from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.descriptions import executable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

"""Launches the full game simulation with 5v5"""

# TODO(@naefjo): Can is there a way we dont have to hardcode these things?
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
            "world_file": "full_game.wbt",
        }.items(),
    )
    launch_description.add_entity(simulation)

    for namespace, tcp_port in team_config.items():
        launch_description.add_entity(
            IncludeLaunchDescription(
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
                    "robot_namespace": namespace,
                    "nao_lola_connection_type": "TCP",
                    "nao_lola_tcp_port": str(tcp_port),
                    "video_device_upper": "127.0.0.1:" + str(tcp_port + 1),
                    "video_device_lower": "127.0.0.1:" + str(tcp_port + 2),
                }.items(),
            )
        )

    return launch_description
