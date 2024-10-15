#include "nomadz_motion_control/walk_engine/step_planner.hpp"

#include <algorithm>

#include "nomadz_core/geometry/projection.hpp"
#include "nomadz_core/math/clamp.hpp"
#include "nomadz_core/math/approx.hpp"
#include "nomadz_core/math/constants.hpp"

#include "nomadz_motion_control/walk_engine/step_pattern_collection.hpp"

using Eigen::Vector2f;
using Eigen::Vector3f;
using nomadz_core::Pose2D;
using nomadz_core::Twist2D;

using nomadz_core::projectTo2D;
using nomadz_core::toAffine2;
using nomadz_core::toPose2D;
using nomadz_core::toTwist2D;
using nomadz_core::toVector3;

namespace side = nomadz_definitions::side;
namespace approx = nomadz_core::approx;

namespace nomadz_motion_control {
  StepPlanner::StepPlanner(const std::shared_ptr<walk_engine_parameters::Params> dyn_params)
      : dyn_params_(std::move(dyn_params)) {
    reset();
  }

  void StepPlanner::reset() {
    upcoming_steps_.clear();
    upcoming_way_points_.clear();
    preview_steps_.clear();
    preview_way_points_.clear();

    measured_speed_ = Twist2D{};
    last_executed_speed_ = Twist2D{};
    last_odometry_ = Pose2D{};

    last_foot_poses_[side::LEFT] = FOOT_DEFAULT_POSE[side::LEFT];
    last_foot_poses_[side::RIGHT] = FOOT_DEFAULT_POSE[side::RIGHT];
  }

  void StepPlanner::setBallPos(const Vector2f& ball_pos) {
    ball_position_ = ball_pos;
  }

  void StepPlanner::setKickRequest(float kick_direction, float kick_power) {
    kick_direction_ = kick_direction;
    kick_power_ = kick_power;
  }

  // Main update loop for the step planner
  void StepPlanner::calcStepPattern(const WalkCoreState& walk_core_state) {
    updateFromCoreState(walk_core_state);
    estimateSpeed();

    switch (walk_core_state.walk_mode) {
    case WalkMode::SPEED:
    default:
      current_executed_speed_ = processSpeedRequest(walk_core_state.target_speed);
      break;
    case WalkMode::PATTERN:
      calcNextGaitPatterns(walk_core_state);
      break;
    }

    step_duration_ = calcPhaseDuration(current_executed_speed_);
    calcPreviewGaitsFromSpeed(current_executed_speed_, step_duration_);
    calcTargetFootPoses();

    // Generate planned steps as output for the walk engine
    planned_steps_.upcoming_steps = upcoming_steps_;
    planned_steps_.upcoming_odometry = current_executed_speed_ * nomadz_core::constants::MOTION_CYCLE_TIME;
    planned_steps_.measured_odometry = measured_speed_ * nomadz_core::constants::MOTION_CYCLE_TIME;
    planned_steps_.planned_speed = current_executed_speed_;
    planned_steps_.step_duration = step_duration_;
    planned_steps_.upcoming_way_points = upcoming_way_points_;
    upcoming_steps_.clear();
    upcoming_way_points_.clear();
    updateHistory();
  }

  void StepPlanner::updateFromCoreState(const WalkCoreState& walk_core_state) {
    swing_side_ = walk_core_state.is_left_phase ? side::LEFT : side::RIGHT;
    support_side_ = walk_core_state.is_left_phase ? side::RIGHT : side::LEFT;
    measured_foot_poses_[side::LEFT] = toPose2D(projectTo2D(walk_core_state.measured_foot_poses[side::LEFT]));
    measured_foot_poses_[side::RIGHT] = toPose2D(projectTo2D(walk_core_state.measured_foot_poses[side::RIGHT]));
    measured_foot_poses_[side::LEFT].x += dyn_params_->foot_origin_offset_x;
    measured_foot_poses_[side::RIGHT].x += dyn_params_->foot_origin_offset_x;
    if (!initialized_) {
      last_measured_foot_poses_ = measured_foot_poses_;
      initialized_ = true;
    }
  }

  void StepPlanner::estimateSpeed() {
    measured_speed_ =
      (last_measured_foot_poses_[last_support_side_] - measured_foot_poses_[last_support_side_]) / step_duration_;
  }

  Twist2D StepPlanner::processSpeedRequest(const Twist2D& target_speed) {
    request_speed_ = toTwist2D(nomadz_core::ellipsoidClamp(toVector3(target_speed), toVector3(MAX_SPEED)));

    Acc2D requested_speed_change = request_speed_ - last_executed_speed_;
    requested_speed_change =
      toTwist2D(nomadz_core::rectClamp(toVector3(requested_speed_change), toVector3(MAX_ACCELERATION)));
    return last_executed_speed_ + requested_speed_change;
  }

  void StepPlanner::calcPreviewGaitsFromSpeed(const Twist2D& requested_speed, const float step_duration) {
    preview_steps_.clear();
    preview_way_points_.clear();
    Step step_i;
    Step prev_step_i;
    Pose2D way_point_i;
    Twist2D support_foot_offset;

    if (upcoming_steps_.empty()) {
      step_i = Step{support_side_, 0.F, last_foot_poses_[support_side_]};
      prev_step_i = Step{swing_side_, 0.F, last_foot_poses_[swing_side_]};
    } else if (upcoming_steps_.size() == 1) {
      step_i = upcoming_steps_.back();
      prev_step_i = Step{support_side_, 0.F, last_foot_poses_[support_side_]};

    } else if (upcoming_steps_.size() >= 2) {
      step_i = upcoming_steps_.back();
      prev_step_i = upcoming_steps_[upcoming_steps_.size() - 2];
    }
    way_point_i = computeOriginFromSteps(step_i, prev_step_i);
    support_foot_offset =
      FOOT_DEFAULT_POSE[support_side_] - toPose2D(toAffine2(way_point_i).inverse() * toAffine2(step_i.leg_pose));

    unsigned int num_additional_steps = std::max(0, MIN_UPCOMING_PATTERN - static_cast<int>(upcoming_steps_.size()));

    Twist2D requested_origin_pose_diff = requested_speed * step_duration;
    Twist2D diff = calcEffectiveOriginPoseDiff(support_foot_offset, requested_origin_pose_diff);
    updateFootPoseLimits(requested_origin_pose_diff);
    for (unsigned int i = 0; i < num_additional_steps; i++) {
      Pose2D next_origin = toPose2D(toAffine2(way_point_i) * toAffine2(Pose2D{} + diff));
      Step next_step = computeNextStepFromOrigin(next_origin, step_i);
      preview_steps_.push_back(next_step);
      preview_way_points_.push_back(next_origin);

      step_i = next_step;

      way_point_i = next_origin;
    }
    upcoming_steps_.insert(upcoming_steps_.end(), preview_steps_.begin(), preview_steps_.end());
    upcoming_way_points_.insert(upcoming_way_points_.end(), preview_way_points_.begin(), preview_way_points_.end());
  }

  void StepPlanner::calcTargetFootPoses() {
    Pose2D next_origin = upcoming_way_points_.at(0);

    Pose2D support_foot_target_pose = last_foot_poses_[support_side_];
    Pose2D swing_foot_target_pose = upcoming_steps_.at(0).leg_pose;

    planned_steps_.target_foot_poses[support_side_] =
      toPose2D(toAffine2(next_origin).inverse() * toAffine2(support_foot_target_pose));
    planned_steps_.target_foot_poses[swing_side_] =
      toPose2D(toAffine2(next_origin).inverse() * toAffine2(swing_foot_target_pose));
  }

  void StepPlanner::updateHistory() {
    last_odometry_ = toPose2D(toAffine2(planned_steps_.target_foot_poses[support_side_]).inverse() *
                              toAffine2(last_foot_poses_[support_side_]));
    planned_steps_.is_current_step_kick = false;
    if (executing_target_pattern_) {
      if (!target_steps_.empty()) {
        target_steps_.erase(target_steps_.begin());
        target_way_points_.erase(target_way_points_.begin());
        if (target_steps_.size() == 1) {
          planned_steps_.is_current_step_kick = true;
          cool_down_counter_ = 5;
        }
      }
      if (target_steps_.empty()) {
        executing_target_pattern_ = false;
      } else {
        for (Step& step : target_steps_) {
          Pose2D updated_leg_pose = toPose2D(toAffine2(last_odometry_).inverse() * toAffine2(step.leg_pose));
          step = Step{step.side, step.step_duration, updated_leg_pose};
        }
        for (Pose2D& way_point : target_way_points_) {
          way_point = toPose2D(toAffine2(last_odometry_).inverse() * toAffine2(way_point));
        }
      }
    }
    if (cool_down_counter_ > 0) {
      cool_down_counter_ -= 1;
    } else {
      cool_down_counter_ = 0;
    }
    last_support_side_ = support_side_;
    last_measured_foot_poses_ = measured_foot_poses_;
    last_foot_poses_ = planned_steps_.target_foot_poses;
    last_executed_speed_ = planned_steps_.planned_speed;
  }

  void StepPlanner::calcNextGaitPatterns(const WalkCoreState& walk_core_state) {
    Pose2D target_frame{ball_position_.x(), ball_position_.y(), kick_direction_};
    if (!executing_target_pattern_ && cool_down_counter_ == 0) {
      target_steps_ = STEP_PATTERN_MAP.at(walk_core_state.step_pattern_type)(target_frame);
      target_way_points_.clear();
      Step first_target_step = target_steps_.at(0);
      float foot_origin_offset_y = (first_target_step.side == side::LEFT) ? -0.05F : 0.05F;
      Pose2D first_target_waypoint = first_target_step.leg_pose;
      first_target_waypoint.y += foot_origin_offset_y;
      if (std::abs(first_target_waypoint.x) < 0.1F && std::abs(first_target_waypoint.y) < 0.03F &&
          std::abs(first_target_waypoint.theta) < 0.1F) {
        if (first_target_step.side != swing_side_) {
          // insert an additional buffer step
          Pose2D buffer_foot_pose = first_target_step.leg_pose;
          buffer_foot_pose.y = last_foot_poses_[support_side_].y + foot_origin_offset_y * 2.F;
          buffer_foot_pose = last_foot_poses_[swing_side_] + (buffer_foot_pose - last_foot_poses_[swing_side_]) / 2.F;
          target_steps_.insert(target_steps_.begin(), Step{swing_side_, 0.25F, buffer_foot_pose});
        }
        executing_target_pattern_ = true;
      } else {
        float rot_offset = std::atan2(first_target_waypoint.y, first_target_waypoint.x);
        Twist2D target_speed{};
        if (std::abs(rot_offset) > 0.45F) {
          target_speed.theta = rot_offset / 0.15F;
        } else {
          target_speed.x = first_target_waypoint.x / 0.15F;
          target_speed.y = first_target_waypoint.y / 0.15F;
          target_speed.theta = rot_offset / 0.15F;
        }
        current_executed_speed_ = processSpeedRequest(target_speed);
      }
    }

    if (executing_target_pattern_) {
      // Update gaits from odometry
      Step last_step = Step{support_side_, 0.25F, last_foot_poses_[support_side_]};
      for (Step step : target_steps_) {
        Pose2D way_point_i = computeOriginFromSteps(step, last_step);
        target_way_points_.push_back(way_point_i);
        last_step = step;
      }
      upcoming_steps_.insert(upcoming_steps_.end(), target_steps_.begin(), target_steps_.end());
      upcoming_way_points_.insert(upcoming_way_points_.end(), target_way_points_.begin(), target_way_points_.end());
      // Approx the speed during
      Twist2D estimated_speed = (upcoming_way_points_.at(0) - Pose2D{}) / 0.25F;
      current_executed_speed_ = (upcoming_way_points_.at(0) - Pose2D{}) / calcPhaseDuration(estimated_speed);
    }
  }

  std::vector<Step> StepPlanner::perStepGenerator(const Step& initial_target_step) {
    std::vector<Step> pre_steps;
    Step step_i;
    step_i.leg_pose =
      Pose2D{} + (initial_target_step.leg_pose - measured_foot_poses_[side::mirror(initial_target_step.side)]);
    pre_steps.push_back(step_i);
    return pre_steps;
  }

  Twist2D StepPlanner::calcEffectiveOriginPoseDiff(const Twist2D& support_foot_offset,
                                                   const Twist2D& requested_origin_pose_diff) {
    Vector2f trainslation_diff{requested_origin_pose_diff.x, requested_origin_pose_diff.y};
    Vector2f translation_offset{-support_foot_offset.x, -support_foot_offset.y};
    Vector2f diff{0.F, 0.F};

    if (approx::isZero(translation_offset.norm())) {
      diff = trainslation_diff;
    } else {
      if (trainslation_diff.norm() < translation_offset.norm()) {
        diff = translation_offset;
      } else {
        diff = translation_offset + trainslation_diff.normalized() * (trainslation_diff.norm() - translation_offset.norm());
      }
    }
    float rotation_diff{};
    if (approx::isZero(support_foot_offset.theta)) {
      rotation_diff = requested_origin_pose_diff.theta;
    } else {
      float projected_rotation_diff =
        requested_origin_pose_diff.theta * (-support_foot_offset.theta) / std::abs(support_foot_offset.theta);
      rotation_diff = (projected_rotation_diff > std::abs(support_foot_offset.theta)) ? requested_origin_pose_diff.theta
                                                                                      : (-support_foot_offset.theta);
    }

    return Twist2D{diff.x(), diff.y(), rotation_diff};
  }

  float StepPlanner::calcPhaseDuration(const Twist2D& requested_speed) {
    return dyn_params_->base_walk_period + requested_speed.x * dyn_params_->walk_period_increase_factor.x +
           requested_speed.y * dyn_params_->walk_period_increase_factor.y +
           requested_speed.theta * dyn_params_->walk_period_increase_factor.theta;
  }

  Pose2D StepPlanner::computeOriginFromSteps(const Step& step1, const Step& step2) {
    return (step1.leg_pose + (step2.leg_pose - step1.leg_pose) / 2.F);
  }

  Step StepPlanner::computeNextStepFromOrigin(const Pose2D& next_origin, const Step& next_support_step) {
    Step next_swing_step_target{};
    next_swing_step_target.step_duration = BASE_WALK_PERIOD;
    next_swing_step_target.side = side::mirror(next_support_step.side);
    const Twist2D unconstrained_leg_pose_b = (next_origin - next_support_step.leg_pose);
    next_swing_step_target.leg_pose =
      next_origin + toTwist2D(nomadz_core::rectClamp(toVector3(unconstrained_leg_pose_b),
                                                     toVector3(foot_pose_lower_limits_[next_swing_step_target.side]),
                                                     toVector3(foot_pose_upper_limits_[next_swing_step_target.side])));
    return next_swing_step_target;
  }

  void StepPlanner::updateFootPoseLimits(const Twist2D& origin_pose_diff) {
    Twist2D abs_pose_diff{std::abs(origin_pose_diff.x), std::abs(origin_pose_diff.y), std::abs(origin_pose_diff.theta)};
    foot_pose_lower_limits_[side::LEFT] =
      Pose2D{-abs_pose_diff.x / 2.F, FOOT_SELF_COLLISION_LIMIT_Y, -abs_pose_diff.theta * FOOT_INWARDS_ROTATION_LIMIT_FACTOR};
    foot_pose_lower_limits_[side::RIGHT] = Pose2D{-abs_pose_diff.x / 2.F,
                                                  FOOT_DEFAULT_POSE[side::RIGHT].y - abs_pose_diff.y,
                                                  -abs_pose_diff.theta * FOOT_OUTWARDS_ROTATION_LIMIT_FACTOR};

    foot_pose_upper_limits_[side::LEFT] = Pose2D{abs_pose_diff.x / 2.F,
                                                 FOOT_DEFAULT_POSE[side::LEFT].y + abs_pose_diff.y,
                                                 abs_pose_diff.theta * FOOT_OUTWARDS_ROTATION_LIMIT_FACTOR};
    foot_pose_upper_limits_[side::RIGHT] =
      Pose2D{abs_pose_diff.x / 2.F, -FOOT_SELF_COLLISION_LIMIT_Y, abs_pose_diff.theta * FOOT_INWARDS_ROTATION_LIMIT_FACTOR};
  }

} // namespace nomadz_motion_control
