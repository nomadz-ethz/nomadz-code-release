from copy import deepcopy
from math import atan2, cos, pi, sin
from time import time

import rclpy
from geometry_msgs.msg import Pose2D

from nomadz_modeling_msgs.msg import BallModel, RobotPose, WorldModel

FOOT_NAME = ["LEFT_FOOT", "RIGHT_FOOT"]
MAX_PLAYER_PER_TEAM = 7
BALL_NAME = "Ball"
BLUE_NAMES = [f"Blue{i+1}" for i in range(MAX_PLAYER_PER_TEAM)]
RED_NAMES = [f"Red{i+1}" for i in range(MAX_PLAYER_PER_TEAM)]


class Supervisor:
    def init(self, webots_node, properties):
        self.__supervisor = webots_node.robot

        self.__ball_node = self.__supervisor.getFromDef(BALL_NAME)

        self.__nodes = {
            key: self.__supervisor.getFromDef(key)
            for key in BLUE_NAMES + RED_NAMES
            if self.__supervisor.getFromDef(key) is not None
        }

        self.__last_valid_world_model = {}
        self.__last_time_stamp = time()
        rclpy.init(args=None)
        self.__node = rclpy.create_node("supervisor")
        self.__node.declare_parameter("valid_robot_pose", True)
        self.__node.declare_parameter("valid_ball_model", True)

        self.__publisher_dict = {}
        self.__world_model_publisher = self.__node.create_publisher(
            WorldModel, "supervisor/world_model", 10
        )
        for name in self.__nodes.keys():
            self.__publisher_dict[name] = self.__node.create_publisher(
                WorldModel, name + "/supervisor/world_model", 10
            )
            self.__last_valid_world_model[name] = WorldModel()

        self.ball_model = BallModel()
        self.robot_pose = RobotPose()

    def normalize_angle(self, angle):
        return atan2(sin(angle), cos(angle))

    def get_pose2d(self, node, mirror=False):
        foot_middle_pose = Pose2D()
        left_foot_node = node.getFromProtoDef(FOOT_NAME[0])
        right_foot_node = node.getFromProtoDef(FOOT_NAME[1])
        foot_middle_pose.x = (
            left_foot_node.getPosition()[0] + right_foot_node.getPosition()[0]
        ) / 2
        foot_middle_pose.y = (
            left_foot_node.getPosition()[1] + right_foot_node.getPosition()[1]
        ) / 2
        foot_middle_pose.theta = atan2(
            left_foot_node.getOrientation()[3] + right_foot_node.getOrientation()[3],
            left_foot_node.getOrientation()[0] + right_foot_node.getOrientation()[0],
        )
        if mirror:
            foot_middle_pose.x = -foot_middle_pose.x
            foot_middle_pose.y = -foot_middle_pose.y
            foot_middle_pose.theta = self.normalize_angle(foot_middle_pose.theta + pi)
        return foot_middle_pose

    def transform_base_world(self, base_pose_w, pose_w):
        pose_b = Pose2D()
        pose_b.x = (pose_w.x - base_pose_w.x) * cos(-base_pose_w.theta) - (
            pose_w.y - base_pose_w.y
        ) * sin(-base_pose_w.theta)
        pose_b.y = (pose_w.x - base_pose_w.x) * sin(-base_pose_w.theta) + (
            pose_w.y - base_pose_w.y
        ) * cos(-base_pose_w.theta)
        pose_b.theta = pose_w.theta - base_pose_w.theta
        return pose_b

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        ball_position = self.__ball_node.getPosition()
        ball_pose_w = Pose2D()
        ball_pose_w.x = ball_position[0]
        ball_pose_w.y = ball_position[1]

        dt = time() - self.__last_time_stamp
        valid_robot_pose = self.__node.get_parameter("valid_robot_pose").value
        valid_ball_model = self.__node.get_parameter("valid_ball_model").value

        for name, node in self.__nodes.items():
            pose_valid = node.getOrientation()[8] > 0.8
            world_model = WorldModel()
            world_model.header.stamp = self.__node.get_clock().now().to_msg()

            team_color = "Blue" if name in BLUE_NAMES else "Red"
            mirror_perspective = team_color == "Red"
            if pose_valid:
                self.robot_pose.header.stamp = self.__node.get_clock().now().to_msg()
                self.robot_pose.pose = self.get_pose2d(node, mirror=mirror_perspective)
                if valid_robot_pose:
                    self.robot_pose.last_valid_time_stamp = (
                        self.__node.get_clock().now().to_msg()
                    )

                self.robot_pose.valid = valid_robot_pose
                self.robot_pose.lost = not valid_robot_pose

                self.ball_model.header.stamp = self.__node.get_clock().now().to_msg()
                ball_pos_w_from_team_perspective = deepcopy(ball_pose_w)
                if team_color == "Red":
                    ball_pos_w_from_team_perspective.x = (
                        -ball_pos_w_from_team_perspective.x
                    )
                    ball_pos_w_from_team_perspective.y = (
                        -ball_pos_w_from_team_perspective.y
                    )
                ball_pose = self.transform_base_world(
                    self.robot_pose.pose, ball_pos_w_from_team_perspective
                )
                self.ball_model.position.x = ball_pose.x
                self.ball_model.position.y = ball_pose.y

                self.ball_model.velocity.x = (
                    self.ball_model.position.x
                    - self.__last_valid_world_model[name].ball_model.position.x
                ) / dt
                self.ball_model.velocity.y = (
                    self.ball_model.position.y
                    - self.__last_valid_world_model[name].ball_model.position.y
                ) / dt
                if valid_ball_model:
                    self.ball_model.last_valid_time_stamp = (
                        self.__node.get_clock().now().to_msg()
                    )

                self.ball_model.valid = valid_ball_model
                self.ball_model.lost = not valid_ball_model

                world_model.robot_pose = self.robot_pose
                world_model.ball_model = self.ball_model

                for other_name, other_node in self.__nodes.items():
                    if name == other_name:
                        continue
                    if team_color in other_name:
                        world_model.teammate_poses.append(
                            self.get_pose2d(other_node, mirror=mirror_perspective)
                        )
                    else:
                        world_model.opponent_poses.append(
                            self.get_pose2d(other_node, mirror=mirror_perspective)
                        )

                self.__last_valid_world_model[name] = world_model
            else:
                world_model = self.__last_valid_world_model[name]
                world_model.robot_pose.valid = False
                world_model.robot_pose.lost = True

                world_model.ball_model.valid = False
                world_model.ball_model.lost = True

            self.__publisher_dict[name].publish(world_model)
        self.__world_model_publisher.publish(world_model)
        self.__last_time_stamp = time()
