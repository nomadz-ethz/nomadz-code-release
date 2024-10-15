#include "nomadz_behavior/helper/path_planner.hpp"

#include <Eigen/Core>

#include "nomadz_core/geometry/angle.hpp"
#include "nomadz_core/geometry/interpolation.hpp"
#include "nomadz_core/math/clamp.hpp"

using Eigen::Vector2f;
using Pose2D = nomadz_core::Pose2D;
using Twist2D = nomadz_core::Twist2D;
using nomadz_core::ellipsoidClamp;
using nomadz_core::toAffine2;
using nomadz_core::toPose2D;
using nomadz_core::toTwist2D;
using nomadz_core::toVector3;
using nomadz_core::angle::normalize;
using nomadz_core::interpolation::rationalBezierInterpolation;

namespace nomadz_behavior {
  Twist2D calculateSpeedFromTargetPose(const Pose2D& current_pose, const Pose2D& target_pose) {
    Twist2D pose_diff = target_pose - current_pose;
    float target_direction = std::atan2(pose_diff.y, pose_diff.x);
    float target_distance = Eigen::Vector2f(pose_diff.x, pose_diff.y).norm();
    std::array<Vector2f, 4> control_points = {Vector2f{0.F, normalize(current_pose.theta - target_direction)},
                                              Vector2f{0.F, 0.F},
                                              Vector2f{target_distance, 0.F},
                                              Vector2f{target_distance, normalize(target_pose.theta - target_direction)}};
    std::array<float, 4> weights = {1.0, 5, 5, 1.0};

    auto next_projected_pose =
      rationalBezierInterpolation<Eigen::Vector2f, 3>(control_points, weights, std::min(1.F, MAX_SPEED.x / target_distance));
    Pose2D next_pose = current_pose + pose_diff * next_projected_pose.x() / target_distance;
    next_pose.theta = normalize(target_direction + next_projected_pose.y());

    Twist2D target_speed = toPose2D(toAffine2(current_pose).inverse() * toAffine2(next_pose)) - Pose2D{};
    const float speed_reduction_factor =
      std::min(1.F, 0.2F + (target_distance / MAX_SPEED.x) + (std::abs(pose_diff.theta) / MAX_SPEED.theta));
    const Twist2D augmented_max_speed{
      MAX_SPEED.x * speed_reduction_factor, MAX_SPEED.y * speed_reduction_factor, MAX_SPEED.theta};
    return toTwist2D(ellipsoidClamp(toVector3(target_speed * 5.F), toVector3(augmented_max_speed)));
  } // namespace nomadz_behavior

} // namespace nomadz_behavior
