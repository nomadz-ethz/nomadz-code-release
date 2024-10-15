#pragma once

#include <Eigen/Core>

#include <geometry_msgs/msg/quaternion.hpp>

#include "nomadz_definitions/limbs.hpp"
#include "nomadz_core/geometry/twist.hpp"
#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_motion_control/walk_engine/planned_steps.hpp"
#include "nomadz_motion_control_msgs/walk_request_enums.hpp"

namespace nomadz_motion_control {

  using WalkMode = nomadz_motion_control_msgs::WalkMode;
  enum class WalkState { STANDING, STARTING, WALKING, STOPPING };
  constexpr float FOOT_ORIGIN_OFFSET_X = 0.012F; // Offset to the center of the foot

  struct WalkCoreState {
    // Walk core states
    WalkMode walk_mode{WalkMode::SPEED};
    PatternType step_pattern_type;

    float phase_time{0.F};
    bool is_left_phase{false};
    WalkState walk_state{WalkState::STANDING};

    // Raw sensor measurements
    float angle_pitch;
    float angle_roll;
    Eigen::Affine3f orientation;
    Eigen::Vector3f gyro;

    // Estimation
    float foot_support{0};
    std::array<bool, nomadz_definitions::side::NUM_SIDES> ground_contacts{true, true};
    nomadz_core::Twist2D estimated_speed{};
    std::array<Eigen::Affine3f, nomadz_definitions::side::NUM_SIDES> measured_foot_poses;

    float step_duration{0.25F};
    std::array<Eigen::Affine3f, nomadz_definitions::side::NUM_SIDES> target_foot_poses =
      {}; // From 2D planned step to 3D in base frame
    nomadz_core::Twist2D target_speed{};
  };
} // namespace nomadz_motion_control
