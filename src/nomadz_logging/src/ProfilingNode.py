#!/usr/bin/python3
import psutil
import rclpy
from rclpy.node import Node

from nomadz_msgs.msg import SystemStats


class ProfilerNode(Node):
    def __init__(self):
        super().__init__("SystemStats")
        self.publisher_ = self.create_publisher(SystemStats, "SystemStats", 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = SystemStats()
        msg.cpu_usage_percent = float(psutil.cpu_percent(1))
        msg.ram_usage_percent = float(psutil.virtual_memory()[2])
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ProfilerNode()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
