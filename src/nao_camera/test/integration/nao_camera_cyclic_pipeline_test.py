import unittest

import launch_testing
import pytest
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes

CAMERA_COMPONENT_NAME = "nao_camera"
CONTAINER_NAME = "cyclic_pipeline_container"


@pytest.mark.launch_test
def generate_test_description():
    container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nao_camera",
                plugin="nao_camera::NaoCamera",
                name=CAMERA_COMPONENT_NAME,
                parameters=[
                    {
                        "device_type": "fake",
                        "image_size": [640, 480],
                        "frame_rate": 30,
                        "camera_info_url": "package://nao_camera/config/def_upper_camera_calib.yaml",
                    },
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="nao_camera",
                plugin="nao_camera::DummyImageProcessor",
                name="image_processor",
                remappings=[
                    ("image", "nao_camera/image"),
                    ("image_processed", "nao_camera/image_processed"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            container,
            ReadyToTest(),
        ]
    )


class TestNaoCameraInitialization(unittest.TestCase):
    def test_initialization(self, proc_output):
        proc_output.assertWaitFor(
            f"Received image with timestamp",
            timeout=3,
        )


@launch_testing.post_shutdown_test()
class TestNaoCameraShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        assertExitCodes(proc_info)
