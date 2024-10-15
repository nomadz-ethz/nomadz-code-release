from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.descriptions import executable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

"""Launches all Nodes of a robot in simulation."""


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_namespace_launch_arg = DeclareLaunchArgument(
        "robot_namespace",
        description="Name which should be given to the namespace of a single robot",
        default_value="",
    )

    proprioception_node = Node(
        package="nomadz_proprioception",
        executable="proprioception_node",
    )

    motion_node = Node(
        package="nomadz_motion_control",
        executable="motion_control_node",
    )

    namespaced_nodes = GroupAction(
        [
            PushRosNamespace(robot_namespace),
            proprioception_node,
            motion_node,
        ],
    )

    return LaunchDescription(
        [
            robot_namespace_launch_arg,
            namespaced_nodes,
        ]
    )
