import time
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from pynput import keyboard
from rclpy.node import Node

from nomadz_motion_control_msgs.msg import MotionRequest


class MotionType(Enum):
    SPECIAL_ACTION = 0
    WALK = 1
    KICK = 2


class WalkMode(Enum):
    SPEED = 0
    PATTERN = 1


class SpecialActionType(Enum):
    PLAY_DEAD = 0
    SIT_DOWN = 1
    GO_UP = 2
    STAND = 3
    STAND_HIGH = 4
    FALL_PROTECTION_BACK = 5
    FALL_PROTECTION_FRONT = 6
    FALL_PROTECTION_SIDE = 7
    GET_UP_BACK_NAO_22 = 8
    GET_UP_FRONT_NAO_22 = 9
    SIT_DOWN_GOALKEEPER = 10


class HeadMotionType(Enum):
    DIRECT = 0
    TARGET = 1


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_control_node")
        self.motion_request_publisher_ = self.create_publisher(
            MotionRequest, "behavior/motion_request", 10
        )
        self.get_logger().info(
            "Keyboard Teleop Node started. Use W, A, S, D to translation the robot, press Q, and E to rotation and use arrow keys to move to head."
        )

        # Dictionary to track key press start times
        self.key_press_start_time = {}
        self.target_speed = Twist()  # Initialize with zero velocities
        self.head_pitch = 0.0
        self.head_yaw = 0.0

        # Speed increment configuration
        self.linear_increment = 0.02  # Linear speed increment per time unit
        self.angular_increment = 0.02  # Angular speed increment per time unit
        self.head_pos_increment = 0.1  # Head position increment per time unit

        self.max_linear_speed = 0.25  # Max linear speed
        self.max_angular_speed = 2.0  # Max angular speed

        self.head_pitch_max = 29.5 * 3.14159 / 180
        self.head_pitch_min = -38.5 * 3.14159 / 180
        self.head_yaw_max = 119.5 * 3.14159 / 180

        # Start listening to keyboard events
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.listener.start()

        self.translation_keys = ["w", "s", "a", "d"]
        self.rotation_keys = ["q", "e"]
        self.head_rotation_keys = [
            keyboard.Key.left,
            keyboard.Key.right,
            keyboard.Key.up,
            keyboard.Key.down,
        ]

    def on_press(self, key):
        """Callback for when a key is pressed."""

        # Calculate the speed based on how long the key has been pressed

        if hasattr(key, "char"):
            key = key.char
            if key not in self.key_press_start_time:
                self.key_press_start_time[key] = time.time()
            elapsed_time = time.time() - self.key_press_start_time[key]
            if key in self.translation_keys:
                speed = elapsed_time * self.linear_increment
                if key == "w":
                    self.target_speed.linear.x = min(
                        self.target_speed.linear.x + speed, self.max_linear_speed
                    )
                elif key == "s":
                    self.target_speed.linear.x = max(
                        self.target_speed.linear.x - speed, -self.max_linear_speed
                    )
                elif key == "a":
                    self.target_speed.linear.y = min(
                        self.target_speed.linear.y + speed, self.max_linear_speed
                    )
                elif key == "d":
                    self.target_speed.linear.y = max(
                        self.target_speed.linear.y - speed, -self.max_linear_speed
                    )
            elif key in self.rotation_keys:
                speed = elapsed_time * self.angular_increment
                if key == "q":
                    self.target_speed.angular.z = min(
                        self.target_speed.angular.z + speed, self.max_angular_speed
                    )
                elif key == "e":
                    self.target_speed.angular.z = max(
                        self.target_speed.angular.z - speed, -self.max_angular_speed
                    )
        else:
            if key not in self.key_press_start_time:
                self.key_press_start_time[key] = time.time()
            elapsed_time = time.time() - self.key_press_start_time[key]
            if key in self.head_rotation_keys:
                pos_diff = elapsed_time * self.head_pos_increment
                if key == keyboard.Key.left:
                    self.head_yaw = min(self.head_yaw + pos_diff, self.head_yaw_max)
                elif key == keyboard.Key.right:
                    self.head_yaw = max(self.head_yaw - pos_diff, -self.head_yaw_max)
                elif key == keyboard.Key.up:
                    self.head_pitch = max(
                        self.head_pitch - pos_diff, self.head_pitch_min
                    )
                elif key == keyboard.Key.down:
                    self.head_pitch = min(
                        self.head_pitch + pos_diff, self.head_pitch_max
                    )

        self.publish_motion_request()

    def on_release(self, key):
        """Callback for when a key is released."""
        if hasattr(key, "char"):
            if key.char in self.key_press_start_time:
                del self.key_press_start_time[key.char]
            if key.char == "r":
                self.target_speed = Twist()
                self.head_pitch = 0.0
                self.head_yaw = 0.0
        else:
            del self.key_press_start_time[key]

        # Publish the zeroed twist to stop movement
        self.publish_motion_request()

    def publish_motion_request(self):
        motion_request = MotionRequest()
        motion_request.motion_type = MotionType.WALK.value
        motion_request.walk_request.walk_mode = WalkMode.SPEED.value
        motion_request.walk_request.target_speed = self.target_speed

        motion_request.head_motion_request.head_motion_type = (
            HeadMotionType.DIRECT.value
        )
        motion_request.head_motion_request.tilt = self.head_pitch
        motion_request.head_motion_request.pan = self.head_yaw
        motion_request.head_motion_request.speed = 0.5
        self.motion_request_publisher_.publish(motion_request)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
