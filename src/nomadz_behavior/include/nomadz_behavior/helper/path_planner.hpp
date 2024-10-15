#pragma once

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_core/geometry/twist.hpp"

namespace nomadz_behavior {
  constexpr nomadz_core::Twist2D MAX_SPEED{0.25F, 0.2F, 2.F};
  nomadz_core::Twist2D calculateSpeedFromTargetPose(const nomadz_core::Pose2D& current_pose,
                                                    const nomadz_core::Pose2D& target_pose);
  class PathPlanner {
  public:
    PathPlanner() = default;

  private:
  };
} // namespace nomadz_behavior
