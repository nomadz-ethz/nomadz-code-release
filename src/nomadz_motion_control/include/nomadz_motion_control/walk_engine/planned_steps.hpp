#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_core/geometry/twist.hpp"
#include "nomadz_definitions/limbs.hpp"
#include "nomadz_motion_control_msgs/walk_request_enums.hpp"

namespace nomadz_motion_control {
  using FootPoses = std::array<nomadz_core::Pose2D, nomadz_definitions::side::NUM_SIDES>;

  using PatternType = nomadz_motion_control_msgs::PatternType;

  struct Step {
    nomadz_definitions::side::Side side;
    float step_duration;
    nomadz_core::Pose2D leg_pose;
  };

  struct PlannedSteps {
    enum StepType { NORMAL, PREPARE, KICK, NUM_STEP_TYPES } step_type;
    std::vector<Step> upcoming_steps;
    std::vector<nomadz_core::Pose2D> upcoming_way_points;
    FootPoses target_foot_poses;

    nomadz_core::Twist2D upcoming_odometry;
    nomadz_core::Twist2D measured_odometry;
    float step_duration;
    nomadz_core::Twist2D planned_speed;
    bool is_leaving_possible;
    bool is_current_step_kick;
  };

} // namespace nomadz_motion_control
