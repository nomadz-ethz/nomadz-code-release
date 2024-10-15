#pragma once

#include <map>
#include <vector>
#include <Eigen/Core>

#include "nomadz_core/geometry/pose.hpp"

#include "nomadz_motion_control/walk_engine/planned_steps.hpp"

namespace nomadz_motion_control {
  constexpr float FOOT_KICK_OFFSET_X = 0.03F;
  constexpr float BALL_RADIUS = 0.0F;

  inline Eigen::Affine2f makeAffine2(float x, float y, float theta) {
    Eigen::Affine2f transform = Eigen::Translation2f(x, y) * Eigen::Rotation2Df(theta);
    return transform;
  }

  std::vector<Step> generateDefaultStepPatterns(const nomadz_core::Pose2D& target_frame);

  std::vector<Step> generateOmniStepPatterns(const nomadz_core::Pose2D& target_frame);

  std::vector<Step> generateCircleAroundPatterns(const nomadz_core::Pose2D& target_frame);

  std::vector<Step> generateAlignmentPatterns(const nomadz_core::Pose2D& target_frame);

  inline const std::map<PatternType, std::function<std::vector<Step>(nomadz_core::Pose2D)>> STEP_PATTERN_MAP = {
    {PatternType::DEFAULT, generateDefaultStepPatterns},
    {PatternType::IN_WALK_KICK, generateOmniStepPatterns},
    {PatternType::ALIGNMENT, generateAlignmentPatterns}};

} // namespace nomadz_motion_control
