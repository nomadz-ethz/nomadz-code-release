#include <vector>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_modeling_msgs/msg/robot_pose.hpp"
#include "nomadz_modeling/landmark_registrator.hpp"
#include "nomadz_modeling/pose_registrator.hpp"
#include "nomadz_modeling/line_registrator.hpp"
#include "nomadz_modeling/self_locator.hpp"

#include "nomadz_vision_msgs/msg/landmark.hpp"
#include "nomadz_vision_msgs/msg/intersection.hpp"
#include "nomadz_vision_msgs/msg/field_line.hpp"

namespace nomadz_modeling {

  /**
   * Convenience function to create a Pose2D msg from a Pose2D object
   */
  geometry_msgs::msg::Pose2D createPose2DMsgFromPose(const nomadz_core::Pose2D& robot_pose);

  /**
   * Convenience function to create a RobotPose ros message from a Pos2D given valid and lost values
   */
  nomadz_modeling_msgs::msg::RobotPose createRobotPoseMsg(const nomadz_core::Pose2D& robot_pose, bool valid, bool lost);

  std::vector<PerceivedLandmark> unpackLandmarks(const std::vector<nomadz_vision_msgs::msg::Landmark>& landmarks_msg);

  std::vector<PerceivedIntersection>
  unpackIntersections(const std::vector<nomadz_vision_msgs::msg::Intersection>& intersections_msg);

  std::vector<PerceivedLine> unpackLines(const std::vector<nomadz_vision_msgs::msg::FieldLine>& field_lines_msg);
} // namespace nomadz_modeling
