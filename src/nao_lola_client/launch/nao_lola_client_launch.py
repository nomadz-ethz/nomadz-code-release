from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.descriptions.executable import LaunchConfiguration
from launch_ros.actions import Node

"""Launches the Nao LoLA client and connects either using UNIX or TCP sockets."""


def generate_launch_description():
    nao_lola_connection_type = LaunchConfiguration("nao_lola_connection_type")
    nao_lola_connection_type_launch_arg = DeclareLaunchArgument(
        "nao_lola_connection_type", default_value="TCP", choices=["UNIX", "TCP"]
    )
    nao_lola_unix_socket_endpoint = LaunchConfiguration("nao_lola_unix_socket_endpoint")
    nao_lola_unix_socket_endpoint_launch_arg = DeclareLaunchArgument(
        "nao_lola_unix_socket_endpoint", default_value="/tmp/robocup"
    )
    nao_lola_tcp_port = LaunchConfiguration("nao_lola_tcp_port")
    nao_lola_tcp_port_launch_arg = DeclareLaunchArgument(
        "nao_lola_tcp_port",
        default_value="10000",
    )
    lola_client_node = Node(
        name="nao_lola_client",
        package="nao_lola_client",
        executable="nao_lola_client",
        parameters=[
            {
                "endpoint.connection_type": nao_lola_connection_type,
                "endpoint.nao_lola_socket_endpoint": nao_lola_unix_socket_endpoint,
                "endpoint.tcp_port": nao_lola_tcp_port,
            }
        ],
    )

    return LaunchDescription(
        [
            nao_lola_connection_type_launch_arg,
            nao_lola_unix_socket_endpoint_launch_arg,
            nao_lola_tcp_port_launch_arg,
            lola_client_node,
        ]
    )
