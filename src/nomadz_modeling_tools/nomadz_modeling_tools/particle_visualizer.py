import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node

from nomadz_modeling_msgs.msg import RegisteredLines, RobotPoseList, WorldModel
from nomadz_vision_msgs.msg import FieldPerceptionData


class Dim:
    field_length = 9.0
    field_width = 6.0
    line_width = 0.05
    penalty_mark_size = 0.1
    goal_area_length = 0.6
    goal_area_width = 2.2
    penalty_area_length = 1.65
    penalty_area_width = 4.0
    penalty_mark_distance = 1.3
    center_circle_diameter = 1.5
    border_strip_width = 0.7
    goal_width = 1.5
    goal_depth = 0.5
    goal_height = 0.8
    goal_post_diameter = 0.1
    center_circle_radius = center_circle_diameter / 2.0

    def __init__(self) -> None:
        pass

    def field_boundaries(self):
        # Half dimensions for easier calculations
        half_field_length = self.field_length / 2
        half_field_width = self.field_width / 2

        # Field boundary vertices
        boundary_vertices = [
            (-half_field_length, -half_field_width),
            (-half_field_length, half_field_width),
            (half_field_length, half_field_width),
            (half_field_length, -half_field_width),
        ]

        # Own goal area vertices
        own_goal_area_vertices = [
            (-half_field_length, -self.goal_area_width / 2),
            (-half_field_length, self.goal_area_width / 2),
            (-half_field_length + self.goal_area_length, self.goal_area_width / 2),
            (-half_field_length + self.goal_area_length, -self.goal_area_width / 2),
        ]

        # Opponent goal area vertices
        opponent_goal_area_vertices = [
            (half_field_length, -self.goal_area_width / 2),
            (half_field_length, self.goal_area_width / 2),
            (half_field_length - self.goal_area_length, self.goal_area_width / 2),
            (half_field_length - self.goal_area_length, -self.goal_area_width / 2),
        ]

        # Own penalty area vertices
        own_penalty_area_vertices = [
            (-half_field_length, -self.penalty_area_width / 2),
            (-half_field_length, self.penalty_area_width / 2),
            (
                -half_field_length + self.penalty_area_length,
                self.penalty_area_width / 2,
            ),
            (
                -half_field_length + self.penalty_area_length,
                -self.penalty_area_width / 2,
            ),
        ]

        # Opponent penalty area vertices
        opponent_penalty_area_vertices = [
            (half_field_length, -self.penalty_area_width / 2),
            (half_field_length, self.penalty_area_width / 2),
            (half_field_length - self.penalty_area_length, self.penalty_area_width / 2),
            (
                half_field_length - self.penalty_area_length,
                -self.penalty_area_width / 2,
            ),
        ]

        # Penalty marks
        own_penalty_mark = (-half_field_length + self.penalty_mark_distance, 0)
        opponent_penalty_mark = (half_field_length - self.penalty_mark_distance, 0)

        # Center circle center
        center_circle_center = (0, 0)

        # Center line (represented by its two endpoints)
        center_line = [(0, -half_field_width), (0, half_field_width)]

        # Center circle (represented by its center and radius)
        center_circle = {
            "center": center_circle_center,
            "radius": self.center_circle_radius,
        }

        return {
            "boundary_vertices": boundary_vertices,
            "own_goal_area_vertices": own_goal_area_vertices,
            "opponent_goal_area_vertices": opponent_goal_area_vertices,
            "own_penalty_area_vertices": own_penalty_area_vertices,
            "opponent_penalty_area_vertices": opponent_penalty_area_vertices,
            "own_penalty_mark": own_penalty_mark,
            "opponent_penalty_mark": opponent_penalty_mark,
            "center_circle": center_circle,
            "center_line": center_line,
        }


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("self_locator_visualizer")
        self._particle_subscriber = self.create_subscription(
            RobotPoseList, "/ukf_debug_states", self.listener_callback, 10
        )

        self._line_subscriber = self.create_subscription(
            RegisteredLines, "/registered_lines", self.line_callback, 10
        )
        self._vision_line_subscriber = self.create_subscription(
            FieldPerceptionData, "/field_perception_data", self.field_line_callback, 10
        )
        self._world_model_sub = self.create_subscription(
            WorldModel, "/modeling/world_model", self.world_model_cb, 10
        )
        plt.ion()

        self.dim = Dim()
        self.fig, (self.ax_body, self.ax_world) = plt.subplots(2, 1, figsize=(8, 16))
        self.ax_body.set_xlim(-3, 3)
        self.ax_body.set_ylim(-3, 5)
        self.ax_body.set_aspect("equal")
        self.draw_field()

        self.first_run = True
        self.plotted_lines_world = []
        self.plotted_lines_body = []

        self.sc_landmark_seen_world = None
        self.sc_landmark_seen_body = None
        self.sc_landmark_registered = None

        self.lines_pose_est = []
        self.lines_pose_est_body = []

        self.lines_model = []

        self.pose_est = None

        self.vision_lines_world = []
        self.vision_lines_body = []
        self.vision_intersections_world = None
        self.vision_intersections_body = None

    def draw_field(self):
        components = self.dim.field_boundaries()

        # Draw boundaries
        boundary = components["boundary_vertices"]
        boundary.append(boundary[0])
        boundary_x, boundary_y = zip(*boundary)
        self.ax_world.plot(boundary_x, boundary_y, color="black")

        # Draw goal areas
        own_goal_area = components["own_goal_area_vertices"]
        own_goal_area.append(own_goal_area[0])
        own_goal_x, own_goal_y = zip(*own_goal_area)
        self.ax_world.plot(own_goal_x, own_goal_y, color="black")

        opponent_goal_area = components["opponent_goal_area_vertices"]
        opponent_goal_area.append(opponent_goal_area[0])
        opponent_goal_x, opponent_goal_y = zip(*opponent_goal_area)
        self.ax_world.plot(opponent_goal_x, opponent_goal_y, color="black")

        # Draw penalty areas
        own_penalty_area = components["own_penalty_area_vertices"]
        own_penalty_area.append(own_penalty_area[0])
        own_penalty_x, own_penalty_y = zip(*own_penalty_area)
        self.ax_world.plot(own_penalty_x, own_penalty_y, color="black")

        opponent_penalty_area = components["opponent_penalty_area_vertices"]
        opponent_penalty_area.append(opponent_penalty_area[0])
        opponent_penalty_x, opponent_penalty_y = zip(*opponent_penalty_area)
        self.ax_world.plot(opponent_penalty_x, opponent_penalty_y, color="black")

        # Draw penalty marks
        own_penalty_mark = components["own_penalty_mark"]
        opponent_penalty_mark = components["opponent_penalty_mark"]
        self.ax_world.scatter(*own_penalty_mark, color="black", s=10)
        self.ax_world.scatter(*opponent_penalty_mark, color="black", s=10)

        # Draw center circle
        center_circle = plt.Circle(
            components["center_circle"]["center"],
            components["center_circle"]["radius"],
            color="black",
            fill=False,
        )
        self.ax_world.add_patch(center_circle)

        # Draw center line
        center_line = components["center_line"]
        center_line_x, center_line_y = zip(*center_line)
        self.ax_world.plot(center_line_x, center_line_y, color="black")

        self.ax_world.set_aspect("equal")
        self.ax_world.set_xlim(
            -self.dim.field_length / 2 - 1, self.dim.field_length / 2 + 1
        )
        self.ax_world.set_ylim(
            -self.dim.field_width / 2 - 1, self.dim.field_width / 2 + 1
        )

    def transform_point(self, point):
        x_new = (
            self.pose_est[0]
            + np.cos(self.pose_est[2]) * point[0]
            - np.sin(self.pose_est[2]) * point[1]
        )

        y_new = (
            self.pose_est[1]
            + np.sin(self.pose_est[2]) * point[0]
            + np.cos(self.pose_est[2]) * point[1]
        )

        return [x_new, y_new]

    def field_line_callback(self, field_perception_data):
        for line in self.vision_lines_world:
            popped_line = line.pop(0)
            popped_line.remove()
        for line in self.vision_lines_body:
            popped_line = line.pop(0)
            popped_line.remove()

        self.vision_lines_world = []
        self.vision_lines_body = []

        if self.pose_est is None:
            return

        for field_line in field_perception_data.field_lines:

            self.vision_lines_body.append(
                self.ax_body.plot(
                    [-field_line.start.y, -field_line.end.y],
                    [field_line.start.x, field_line.end.x],
                    c="purple",
                )
            )

            start_world = self.transform_point([field_line.start.x, field_line.start.y])
            end_world = self.transform_point([field_line.end.x, field_line.end.y])

            self.vision_lines_world.append(
                self.ax_world.plot(
                    [start_world[0], end_world[0]],
                    [start_world[1], end_world[1]],
                    c="purple",
                )
            )

        if len(field_perception_data.intersections) == 0:
            if self.vision_intersections_body is not None:
                self.vision_intersections_body.set_offsets(
                    np.c_[np.array(-1000), np.array(-1000)]
                )
            if self.vision_intersections_world is not None:
                self.vision_intersections_world.set_offsets(
                    np.c_[np.array(-1000), np.array(-1000)]
                )

        for intersection in field_perception_data.intersections:
            if self.vision_intersections_body is None:
                self.vision_intersections_body = self.ax_body.scatter(
                    -intersection.position.y,
                    intersection.position.x,
                    c="purple",
                    marker="P",
                )
            else:
                self.vision_intersections_body.set_offsets(
                    np.c_[
                        np.array(-intersection.position.y),
                        np.array(intersection.position.x),
                    ]
                )

            point_world = self.transform_point(
                [intersection.position.x, intersection.position.y]
            )

            if self.vision_intersections_world is None:
                self.vision_intersections_world = self.ax_world.scatter(
                    point_world[0], point_world[1], c="purple", marker="P"
                )
            else:
                self.vision_intersections_world.set_offsets(
                    np.c_[np.array(point_world[0]), np.array(point_world[1])]
                )

    def world_model_cb(self, world_model_msg: WorldModel):
        self.pose_est = [
            world_model_msg.robot_pose.pose.x,
            world_model_msg.robot_pose.pose.y,
            world_model_msg.robot_pose.pose.theta,
        ]
        for line in self.lines_model:
            popped_line = line.pop(0)
            popped_line.remove()

        self.lines_model = []

        pose_x = world_model_msg.robot_pose.pose.x
        pose_y = world_model_msg.robot_pose.pose.y
        pose_x_end = world_model_msg.robot_pose.pose.x + 0.1 * np.cos(
            world_model_msg.robot_pose.pose.theta
        )
        pose_y_end = world_model_msg.robot_pose.pose.y + 0.1 * np.sin(
            world_model_msg.robot_pose.pose.theta
        )

        self.lines_model.append(
            self.ax_world.plot(
                [pose_x, pose_x_end],
                [pose_y, pose_y_end],
                c="g",
                linewidth=5,
            )
        )

    def line_callback(self, registered_lines: RegisteredLines):
        for line in self.plotted_lines_world:
            popped_line = line.pop(0)
            popped_line.remove()
        for line in self.plotted_lines_body:
            popped_line = line.pop(0)
            popped_line.remove()

        if self.pose_est is None:
            return

        self.plotted_lines_world = []
        self.plotted_lines_body = []

        for line in registered_lines.seen_lines:
            self.plotted_lines_body.append(
                self.ax_body.plot(
                    [-line.start.y, -line.end.y],
                    [line.start.x, line.end.x],
                    c="b",
                )
            )

            start = self.transform_point([line.start.x, line.start.y])
            end = self.transform_point([line.end.x, line.end.y])
            self.plotted_lines_world.append(
                self.ax_world.plot(
                    [start[0], end[0]],
                    [start[1], end[1]],
                    c="b",
                )
            )

        for line in registered_lines.registered_lines:
            line = self.ax_world.plot(
                [line.start.x, line.end.x],
                [line.start.y, line.end.y],
                c="r",
            )
            self.plotted_lines_world.append(line)

        seen_landmarks_world = [
            self.transform_point([landmark.x, landmark.y])
            for landmark in registered_lines.seen_landmarks
        ]
        seen_landmarks_x_coords_world = [
            landmark[0] for landmark in seen_landmarks_world
        ]
        seen_landmarks_y_coords_world = [
            landmark[1] for landmark in seen_landmarks_world
        ]
        seen_landmarks_x_coords_body = [
            landmark.x for landmark in registered_lines.seen_landmarks
        ]
        seen_landmarks_y_coords_body = [
            -landmark.y for landmark in registered_lines.seen_landmarks
        ]

        registered_landmarks_x_coords = [
            landmark.x for landmark in registered_lines.registered_landmarks
        ]
        registered_landmarks_y_coords = [
            landmark.y for landmark in registered_lines.registered_landmarks
        ]

        if (
            len(seen_landmarks_x_coords_world) == 0
            or len(registered_landmarks_x_coords) == 0
        ):
            if self.sc_landmark_seen_world is not None:
                self.sc_landmark_seen_world.set_offsets(
                    np.c_[np.array(-1000), np.array(-1000)]
                )
                self.sc_landmark_registered.set_offsets(
                    np.c_[np.array(-1000), np.array(-1000)]
                )
            if self.sc_landmark_seen_body is not None:
                self.sc_landmark_seen_body.set_offsets(
                    np.c_[np.array(-1000), np.array(-1000)]
                )

        if self.sc_landmark_seen_world is None:
            self.sc_landmark_seen_world = self.ax_world.scatter(
                seen_landmarks_x_coords_world,
                seen_landmarks_y_coords_world,
                c="b",
                marker="P",
            )
            self.sc_landmark_registered = self.ax_world.scatter(
                registered_landmarks_x_coords,
                registered_landmarks_y_coords,
                c="r",
                marker="P",
            )
        else:
            self.sc_landmark_seen_world.set_offsets(
                np.c_[
                    np.array(seen_landmarks_x_coords_world),
                    np.array(seen_landmarks_y_coords_world),
                ]
            )
            self.sc_landmark_registered.set_offsets(
                np.c_[
                    np.array(registered_landmarks_x_coords),
                    np.array(registered_landmarks_y_coords),
                ]
            )
        if self.sc_landmark_seen_body is None:
            self.sc_landmark_seen_body = self.ax_body.scatter(
                seen_landmarks_y_coords_body,
                seen_landmarks_x_coords_body,
                c="b",
                marker="P",
            )
        else:
            self.sc_landmark_seen_body.set_offsets(
                np.c_[
                    np.array(seen_landmarks_y_coords_body),
                    np.array(seen_landmarks_x_coords_body),
                ]
            )

    def listener_callback(self, robot_pose_list: RobotPoseList):
        for line in self.lines_pose_est:
            popped_line = line.pop(0)
            popped_line.remove()

        self.lines_pose_est = []

        particle_x_coords = []
        particle_y_coords = []
        particle_end_x = []
        particle_end_y = []
        for pose in robot_pose_list.robot_poses:
            particle_x_coords.append(pose.x)
            particle_y_coords.append(pose.y)
            particle_end_x.append(pose.x + 0.1 * np.cos(pose.theta))
            particle_end_y.append(pose.y + 0.1 * np.sin(pose.theta))

        for i in range(len(particle_x_coords)):
            self.lines_pose_est.append(
                self.ax_world.plot(
                    [particle_x_coords[i], particle_end_x[i]],
                    [particle_y_coords[i], particle_end_y[i]],
                    c="b",
                )
            )

        self.fig.canvas.draw_idle()
        self.fig.canvas.start_event_loop(0.01667)


def main(args=None):
    matplotlib.use("Qt5agg")
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
