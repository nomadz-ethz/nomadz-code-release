#include "nomadz_modeling/ros_conversion.hpp"

#include <Eigen/Core>

namespace nomadz_modeling {

  geometry_msgs::msg::Pose2D createPose2DMsgFromPose(const nomadz_core::Pose2D& robot_pose) {

    geometry_msgs::msg::Pose2D robot_pose_msg;
    robot_pose_msg.x = robot_pose.x;
    robot_pose_msg.y = robot_pose.y;
    robot_pose_msg.theta = robot_pose.theta;
    return robot_pose_msg;
  }

  nomadz_modeling_msgs::msg::RobotPose createRobotPoseMsg(const nomadz_core::Pose2D& robot_pose, bool valid, bool lost) {

    nomadz_modeling_msgs::msg::RobotPose best_pose_msg;
    best_pose_msg.pose = createPose2DMsgFromPose(robot_pose);
    best_pose_msg.valid = valid;
    best_pose_msg.lost = lost;
    return best_pose_msg;
  }

  std::vector<PerceivedLandmark> unpackLandmarks(const std::vector<nomadz_vision_msgs::msg::Landmark>& landmarks_msg) {
    std::vector<PerceivedLandmark> landmarks;
    for (const nomadz_vision_msgs::msg::Landmark& landmark_msg : landmarks_msg) {
      PerceivedLandmark landmark;
      landmark.percept << static_cast<float>(landmark_msg.position.x), static_cast<float>(landmark_msg.position.y);
      landmark.cov_percept << static_cast<float>(landmark_msg.covariance[0]), static_cast<float>(landmark_msg.covariance[1]),
        static_cast<float>(landmark_msg.covariance[2]), static_cast<float>(landmark_msg.covariance[3]);
      landmark.landmark_type = static_cast<LandmarkType>(landmark_msg.type);
      landmarks.push_back(landmark);
    }
    return landmarks;
  }

  std::vector<PerceivedIntersection>
  unpackIntersections(const std::vector<nomadz_vision_msgs::msg::Intersection>& intersections_msg) {
    std::vector<PerceivedIntersection> intersections;
    for (const nomadz_vision_msgs::msg::Intersection& intersection_msg : intersections_msg) {
      PerceivedIntersection intersection;
      intersection.percept << static_cast<float>(intersection_msg.position.x),
        static_cast<float>(intersection_msg.position.y);
      intersection.dir << static_cast<float>(intersection_msg.direction.x), static_cast<float>(intersection_msg.direction.y);
      intersection.cov_percept << static_cast<float>(intersection_msg.covariance[0]),
        static_cast<float>(intersection_msg.covariance[1]), static_cast<float>(intersection_msg.covariance[2]),
        static_cast<float>(intersection_msg.covariance[3]);
      intersection.intersection_type = static_cast<IntersectionType>(intersection_msg.type);
      intersections.push_back(intersection);
    }
    return intersections;
  }

  std::vector<PerceivedLine> unpackLines(const std::vector<nomadz_vision_msgs::msg::FieldLine>& field_lines_msg) {
    std::vector<PerceivedLine> lines;
    for (const nomadz_vision_msgs::msg::FieldLine& field_line_msg : field_lines_msg) {
      PerceivedLine line;
      line.percept.from << static_cast<float>(field_line_msg.start.x), static_cast<float>(field_line_msg.start.y);
      line.percept.to << static_cast<float>(field_line_msg.end.x), static_cast<float>(field_line_msg.end.y);
      line.cov_percept << static_cast<float>(field_line_msg.covariance[0]), static_cast<float>(field_line_msg.covariance[1]),
        static_cast<float>(field_line_msg.covariance[2]), static_cast<float>(field_line_msg.covariance[3]);
      lines.push_back(line);
    }
    return lines;
  }
} // namespace nomadz_modeling
