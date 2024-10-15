import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

ROBOT_CONFIG_DIR = Path("/home/nao/config")


def generate_launch_description():
    nao_camera_config_dir = Path(get_package_share_directory("nao_camera")) / "config"

    camera_settings_file_path = (
        nao_camera_config_dir / "camera_settings.yaml"
    ).resolve(strict=True)
    upper_camera_config_file_path = (
        nao_camera_config_dir / "nao_upper_camera_config.yaml"
    ).resolve(strict=True)
    lower_camera_config_file_path = (
        nao_camera_config_dir / "nao_lower_camera_config.yaml"
    ).resolve(strict=True)

    upper_camera_calib_file_path = ROBOT_CONFIG_DIR / "upper_camera_calib.yaml"
    if not upper_camera_calib_file_path.is_file():
        print(f"Upper camera calibration file not found, copying default")
        ROBOT_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        shutil.copy(
            nao_camera_config_dir / "def_upper_camera_calib.yaml",
            upper_camera_calib_file_path,
        )

    lower_camera_calib_file_path = ROBOT_CONFIG_DIR / "lower_camera_calib.yaml"
    if not lower_camera_calib_file_path.is_file():
        print(f"Lower camera calibration file not found, copying default")
        ROBOT_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        shutil.copy(
            nao_camera_config_dir / "def_lower_camera_calib.yaml",
            lower_camera_calib_file_path,
        )

    upper_camera_container = ComposableNodeContainer(
        name="upper_camera_container",
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
                    {"camera_info_url": f"file://{str(upper_camera_calib_file_path)}"},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="nomadz_image_processing",
                plugin="nomadz_image_processing::ImageProcessor",
                name="upper_camera_image_processor",
                remappings=[
                    ("image", "upper_camera/image"),
                    ("processed_image", "upper_camera/image_processed"),
                    ("camera_info", "upper_camera/camera_info"),
                    ("image_markers", "upper_camera/image_markers"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    lower_camera_container = ComposableNodeContainer(
        name="lower_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nao_camera",
                plugin="nao_camera::NaoCamera",
                name="lower_camera",
                parameters=[
                    camera_settings_file_path,
                    lower_camera_config_file_path,
                    {"camera_info_url": f"file://{str(lower_camera_calib_file_path)}"},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="nomadz_image_processing",
                plugin="nomadz_image_processing::ImageProcessor",
                name="lower_camera_image_processor",
                remappings=[
                    ("image", "lower_camera/image"),
                    ("processed_image", "lower_camera/image_processed"),
                    ("camera_info", "lower_camera/camera_info"),
                    ("image_markers", "lower_camera/image_markers"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    proprioception_node = Node(
        package="nomadz_proprioception",
        executable="proprioception_node",
    )

    motion_control_node = Node(
        package="nomadz_motion_control",
        executable="motion_control_node",
    )

    container_helper = ComposableNodeContainer(
        name="helper_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nomadz_behavior",
                plugin="nomadz_behavior::EgoStatusProvider",
                name="ego_status_provider",
            ),
            ComposableNode(
                package="nomadz_behavior",
                plugin="nomadz_behavior::GameStatusProvider",
                name="game_status_provider",
            ),
            ComposableNode(
                package="nomadz_led_control",
                plugin="nomadz_led_control::LEDHandler",
                name="led_handler",
            ),
            ComposableNode(
                package="nomadz_audio_processing",
                plugin="nomadz_audio_processing::AudioProcessor",
                name="nomadz_audio_processor",
            ),
        ],
        output="screen",
    )
    behavior_node = Node(
        package="nomadz_behavior",
        executable="behavior_node",
    )

    manual_control_node = Node(
        package="nomadz_teleop",
        executable="keyboard_teleop_node",
    )

    communication_node = Node(
        package="nomadz_communication",
        executable="udp_communicator_node",
    )

    modeling_node = Node(
        package="nomadz_modeling",
        executable="modeling_node",
    )

    return LaunchDescription(
        [
            proprioception_node,
            motion_control_node,
            container_helper,
            communication_node,
            upper_camera_container,
            lower_camera_container,
            modeling_node,
            behavior_node,
            # manual_control_node,
        ]
    )
