#pragma once

#include <Eigen/Core>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_motion_control_msgs/motion_request_enums.hpp"

namespace nomadz_behavior {
  using nomadz_motion_control_msgs::HeadMotionType;
  using nomadz_motion_control_msgs::KickType;
  using nomadz_motion_control_msgs::MotionType;
  using nomadz_motion_control_msgs::PatternType;
  using nomadz_motion_control_msgs::SpecialActionType;
  using nomadz_motion_control_msgs::WalkMode;

  struct WalkRequest {
    WalkMode mode;
    nomadz_core::Twist2D target_speed;
    nomadz_core::Pose2D target_pose;
    PatternType pattern_type;
  };

  struct KickRequest {
    KickType kick_type;
    bool mirror;
    float kick_power;
    float kick_direction;
  };

  struct SpecialActionRequest {
    SpecialActionType special_action_type;
    bool mirror;
  };

  struct HeadMotionRequest {
    HeadMotionType head_motion_type;
    float pan;
    float tilt;
    Eigen::Vector3f target;
  };

  struct MotionRequest {
    MotionType motion_type;
    WalkRequest walk_request;
    KickRequest kick_request;
    SpecialActionRequest special_action_request;
  };

  struct MotionInfo {
    MotionRequest executed_motion_request;
    HeadMotionRequest executed_head_motion_request;
    bool is_motion_done;
    bool is_head_motion_done;
  };

} // namespace nomadz_behavior
