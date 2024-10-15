from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher, WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory("nomadz_webots_controller")
    supervisor_description_path = Path(package_dir) / Path(
        "resource/supervisor_plugin.urdf"
    )
    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [package_dir, "worlds", LaunchConfiguration("world_file")]
        ),
    )

    supervisor = WebotsController(
        robot_name="Supervisor",
        parameters=[
            {"robot_description": supervisor_description_path},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_file",
                description="Name of the world file to load.",
                choices=[
                    "nao_robocup.wbt",
                    "full_game.wbt",
                    "full_game_without_lola.wbt",
                ],
                default_value="nao_robocup.wbt",
            ),
            webots,
            supervisor,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
