from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="dummy_image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nao_camera",
                plugin="nao_camera::DummyImageProcessor",
                name="lower_camera_processor",
                remappings=[
                    ("image", "lower_camera/image"),
                    ("image_processed", "lower_camera/image_processed"),
                ],
            ),
            ComposableNode(
                package="nao_camera",
                plugin="nao_camera::DummyImageProcessor",
                name="upper_camera_processor",
                remappings=[
                    ("image", "lower_camera/image"),
                    ("image_processed", "upper_camera/image_processed"),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
