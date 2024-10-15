from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config_dir_path = Path(get_package_share_directory("nao_camera")).joinpath("config")
    lower_camera_config_file_path = config_dir_path.joinpath(
        "sim_lower_camera_config.yaml"
    ).resolve(strict=True)
    upper_camera_config_file_path = config_dir_path.joinpath(
        "sim_upper_camera_config.yaml"
    ).resolve(strict=True)

    video_device_upper = LaunchConfiguration("video_device_upper")
    video_device_upper_launch_arg = DeclareLaunchArgument(
        "video_device_upper", default_value="127.0.0.1:10001"
    )

    video_device_lower = LaunchConfiguration("video_device_lower")
    video_device_lower_launch_arg = DeclareLaunchArgument(
        "video_device_lower", default_value="127.0.0.1:10002"
    )

    container = ComposableNodeContainer(
        name="sim_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nao_camera",
                plugin="nao_camera::NaoCamera",
                name="lower_camera",
                parameters=[
                    lower_camera_config_file_path,
                    {
                        "video_device": video_device_lower,
                        "camera_info_url": "package://nao_camera/config/def_lower_camera_calib.yaml",
                    },
                ],
            ),
            ComposableNode(
                package="nao_camera",
                plugin="nao_camera::NaoCamera",
                name="upper_camera",
                parameters=[
                    upper_camera_config_file_path,
                    {
                        "video_device": video_device_upper,
                        "camera_info_url": "package://nao_camera/config/def_upper_camera_calib.yaml",
                    },
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            video_device_upper_launch_arg,
            video_device_lower_launch_arg,
            container,
        ]
    )
