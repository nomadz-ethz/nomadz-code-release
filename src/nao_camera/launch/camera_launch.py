from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config_dir_path = Path(get_package_share_directory("nao_camera")).joinpath("config")

    upper_camera_config_file_path = config_dir_path.joinpath(
        "nao_upper_camera_config.yaml"
    ).resolve(strict=True)

    lower_camera_config_file_path = config_dir_path.joinpath(
        "nao_lower_camera_config.yaml"
    ).resolve(strict=True)

    camera_settings_file_path = config_dir_path.joinpath(
        "camera_settings.yaml"
    ).resolve(strict=True)

    upper_camera_info_url = "package://nao_camera/config/def_upper_camera_calib.yaml"
    lower_camera_info_url = "package://nao_camera/config/def_lower_camera_calib.yaml"

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nao_camera",
                plugin="nao_camera::NaoCamera",
                name="upper_camera",
                parameters=[
                    camera_settings_file_path,
                    upper_camera_config_file_path,
                    {
                        "camera_info_url": upper_camera_info_url,
                    },
                ],
            ),
            ComposableNode(
                package="nao_camera",
                plugin="nao_camera::NaoCamera",
                name="lower_camera",
                parameters=[
                    camera_settings_file_path,
                    lower_camera_config_file_path,
                    {
                        "camera_info_url": lower_camera_info_url,
                    },
                ],
            ),
        ],
        output="both",
    )

    return LaunchDescription([container])
