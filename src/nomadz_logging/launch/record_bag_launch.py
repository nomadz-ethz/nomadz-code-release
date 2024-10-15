import datetime
import socket
from pathlib import Path
from typing import Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def get_default_topics_config_file_path() -> Path:
    return (
        Path(get_package_share_directory("nomadz_logging"))
        / "config"
        / "default_topics.yaml"
    )


def get_default_bag_path():
    current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    return Path.cwd() / "bags" / "default" / f"nao_{current_time}"


def read_topics_from_file(topics_file: Path) -> List[str]:
    with open(topics_file, "r") as file:
        config: Dict = yaml.safe_load(file)

    return config.get("topics", [])


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    bag_path_launch_arg = DeclareLaunchArgument(
        "bag_path",
        default_value=str(get_default_bag_path),
    )

    topics = read_topics_from_file(get_default_topics_config_file_path())
    for topic in topics:
        print(f"Recording topic: {topic}")

    spawn_bag_recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--output", bag_path] + topics,
        output="screen",
    )

    return LaunchDescription(
        [
            bag_path_launch_arg,
            spawn_bag_recorder,
        ]
    )
