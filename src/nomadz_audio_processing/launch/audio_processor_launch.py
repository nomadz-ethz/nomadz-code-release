from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="audio_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="nomadz_audio_processing",
                        plugin="nomadz_audio_processing::AudioProcessor",
                        name="audio_processor",
                    ),
                ],
                output="screen",
            ),
        ]
    )
