from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher, WebotsLauncher

team_config = [
    "Blue1",
    "Blue2",
    "Blue3",
    # "Blue4",
    # "Blue5",
    "Red1",
    "Red2",
    "Red3",
    # "Red4",
    # "Red5",
]


def generate_launch_description():
    package_dir = get_package_share_directory("nomadz_webots_controller")
    nao_description_path = Path(package_dir) / Path("resource/NaoV6_plugin.urdf")
    launch_description = LaunchDescription()
    for team_name in team_config:
        launch_description.add_entity(
            WebotsController(
                robot_name=team_name,
                parameters=[
                    {
                        "robot_description": nao_description_path,
                    },
                ],
            )
        )
    return launch_description
