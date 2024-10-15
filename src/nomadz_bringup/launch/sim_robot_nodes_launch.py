from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.descriptions import executable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

"""Launches all Nodes of a robot in simulation."""


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_namespace_launch_arg = DeclareLaunchArgument(
        "robot_namespace",
        description="Name which should be given to the namespace of a single robot",
        default_value="",
    )

    nao_lola_connection_type = LaunchConfiguration("nao_lola_connection_type")
    nao_lola_connection_type_launch_arg = DeclareLaunchArgument(
        "nao_lola_connection_type",
        default_value="TCP",
        choices=["UNIX", "TCP"],
        description="Specifies the Type of socket to launch",
    )
    nao_lola_tcp_port = LaunchConfiguration("nao_lola_tcp_port")
    nao_lola_tcp_port_launch_arg = DeclareLaunchArgument(
        "nao_lola_tcp_port", default_value="10000"
    )

    video_device_upper = LaunchConfiguration("video_device_upper")
    video_device_upper_launch_arg = DeclareLaunchArgument(
        "video_device_upper", default_value="127.0.0.1:10001"
    )

    video_device_lower = LaunchConfiguration("video_device_lower")
    video_device_lower_launch_arg = DeclareLaunchArgument(
        "video_device_lower", default_value="127.0.0.1:10002"
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nao_camera"), "launch", "sim_camera_launch.py"]
            )
        ),
        launch_arguments={
            "video_device_upper": video_device_upper,
            "video_device_lower": video_device_lower,
        }.items(),
    )

    nao_lola_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nao_lola_client"),
                    "launch",
                    "nao_lola_client_launch.py",
                ]
            )
        ),
        launch_arguments={
            "nao_lola_connection_type": nao_lola_connection_type,
            "nao_lola_tcp_port": nao_lola_tcp_port,
        }.items(),
    )

    proprioception_node = Node(
        package="nomadz_proprioception",
        executable="proprioception_node",
    )

    motion_control_node = Node(
        package="nomadz_motion_control",
        executable="motion_control_node",
    )

    communication_node = Node(
        package="nomadz_communication",
        executable="udp_communicator_node",
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

    ukf_publish_particles = LaunchConfiguration("ukf_publish_particles")
    ukf_publish_particles_arg = DeclareLaunchArgument(
        "ukf_publish_particles",
        default_value="true",
    )

    modeling_node = Node(
        package="nomadz_modeling",
        executable="modeling_node",
        parameters=[
            {
                "publish_ukf_debug": ukf_publish_particles,
            }
        ],
        output="screen",
    )

    namespaced_nodes = GroupAction(
        [
            PushRosNamespace(robot_namespace),
            camera_launch,
            nao_lola_client,
            proprioception_node,
            motion_control_node,
            container_helper,
            communication_node,
            modeling_node,
            behavior_node,
            # manual_control_node,
        ],
    )

    return LaunchDescription(
        [
            robot_namespace_launch_arg,
            nao_lola_connection_type_launch_arg,
            nao_lola_tcp_port_launch_arg,
            video_device_upper_launch_arg,
            video_device_lower_launch_arg,
            ukf_publish_particles_arg,
            namespaced_nodes,
        ]
    )
