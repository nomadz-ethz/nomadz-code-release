#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "nomadz_core/geometry/twist.hpp"
#include "nomadz_definitions/limbs.hpp"

#include "nomadz_motion_control/joint_requests.hpp"

namespace nomadz_motion_control {

  struct StepTraj {
    enum TrajType { CURRENT_OFFSET, INITIAL_OFFSET, NUM_OF_TRAJ_TYPES };

    Eigen::Affine3f foot[NUM_OF_TRAJ_TYPES];
    Eigen::Affine3f foot_final;
  };

  struct WalkGeneratorData {
    JointRequests joint_request;          /**< The calculated joint angles. */
    nomadz_core::Twist2D odometry_offset; /**< The relative motion in this frame is returned here (in mm and radians). */
  };

} // namespace nomadz_motion_control
