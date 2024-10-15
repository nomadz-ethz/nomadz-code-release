
#include "nomadz_motion_control/walk_engine/walk_generator.hpp"

#include <fstream>
#include <filesystem>

#include <Eigen/Geometry>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_core/geometry/rotation.hpp"
#include "nomadz_core/math/constants.hpp"
#include "nomadz_definitions/joint_indexes.hpp"
#include "nomadz_kinematics/inverse_kinematics.hpp"

#include "nomadz_motion_control/interpolation.hpp"

namespace joint_indexes = nomadz_definitions::joint_indexes;
namespace side = nomadz_definitions::side;
namespace fs = std::filesystem;
namespace constants = nomadz_core::constants;
using nomadz_core::Twist2D;

namespace nomadz_motion_control {

  WalkGenerator::WalkGenerator(const std::shared_ptr<walk_engine_parameters::Params> dyn_params)
      : dyn_params_(std::move(dyn_params)) {
    for (int i = 0; i < side::NUM_SIDES; ++i) {
      float swing_sign = i == side::LEFT ? 1.F : -1.F;
      default_foot_poses_[i].translation() = Eigen::Vector3f{-dyn_params_->foot_origin_offset_x, swing_sign * 0.05F, -0.26F};
      default_foot_poses_[i].linear() = Eigen::Matrix3f::Identity();
      step_traj_[i].foot[StepTraj::INITIAL_OFFSET] = default_foot_poses_[i];
      step_traj_[i].foot[StepTraj::CURRENT_OFFSET] = default_foot_poses_[i];
      step_traj_[i].foot_final = default_foot_poses_[i];
      prev_target_foot_poses_[i] = default_foot_poses_[i];
      prev_measured_foot_poses_[i] = default_foot_poses_[i];
    }
  }

  void WalkGenerator::generateJointRequests(const WalkCoreState& walk_core_state) {
    updateFromCoreState(walk_core_state);
    generateStepTrajectory(walk_core_state, FINAL_TRAJECTORY_TYPE);
    updateLegJoints(walk_core_state);
    setStiffness();
    applyGyroBalance(walk_core_state);
    nomadz_kinematics::inverseTorsoFromFoot(
      step_traj_[side::LEFT].foot_final, step_traj_[side::RIGHT].foot_final, joint_requests_.positions.values);
    // Ignore joints used by arm motion engine
    for (int i = joint_indexes::R_SHOULDER_PITCH; i <= joint_indexes::R_WRIST_YAW; ++i) {
      joint_requests_.joint_ignore[i] = true;
    }
    for (int i = joint_indexes::L_SHOULDER_PITCH; i <= joint_indexes::L_WRIST_YAW; i++) {
      joint_requests_.joint_ignore[i] = true;
    }
    walk_generator_data_.odometry_offset = calcOdometryOffset(walk_core_state);
    walk_generator_data_.joint_request = joint_requests_;
  }

  void WalkGenerator::updateFromCoreState(const WalkCoreState& walk_core_state) {
    if (walk_core_state.phase_time == 0.F) {
      for (int i = 0; i < side::NUM_SIDES; ++i) {
        step_traj_[i].foot[StepTraj::INITIAL_OFFSET] = prev_target_foot_poses_[i];
        prev_target_foot_poses_[i] = walk_core_state.target_foot_poses[i];
      }
    }
    swing_side_ = walk_core_state.is_left_phase ? side::LEFT : side::RIGHT;
    support_side_ = walk_core_state.is_left_phase ? side::RIGHT : side::LEFT;
  }

  Twist2D WalkGenerator::calcOdometryOffset(const WalkCoreState& walk_core_state) {
    Eigen::Affine3f pose_diff =
      prev_measured_foot_poses_[support_side_] * walk_core_state.measured_foot_poses[support_side_].inverse();
    Eigen::Matrix3f rotation_diff = pose_diff.linear();
    Twist2D odometry_offset{
      pose_diff.translation().x(), pose_diff.translation().y(), nomadz_core::getRotationRelativeZAngle(rotation_diff)};

    for (int i = 0; i < side::NUM_SIDES; ++i) {
      prev_measured_foot_poses_[i] = walk_core_state.measured_foot_poses[i];
    }
    return odometry_offset;
  }

  void WalkGenerator::calcGyroOffset(const WalkCoreState& walk_core_state) {
    Eigen::Vector2f angular_pos = Eigen::Vector2f(walk_core_state.angle_pitch, walk_core_state.angle_roll);
    Eigen::Vector2f angular_vel = Eigen::Vector2f(walk_core_state.gyro[1], -walk_core_state.gyro[0]);
    gyro_correction_states_[2] =
      angular_pos * dyn_params_->gyro_compensation_p_gain + angular_vel * dyn_params_->gyro_compensation_d_gain;
    gyro_correction_states_[1] += gyro_correction_states_[2] * constants::MOTION_CYCLE_TIME;
    gyro_correction_states_[0] += (gyro_correction_states_[1] * constants::MOTION_CYCLE_TIME +
                                   gyro_correction_states_[2] * std::pow(constants::MOTION_CYCLE_TIME, 2));
  }

  void WalkGenerator::applyGyroBalance(const WalkCoreState& walk_core_state) {
    filtered_gyro_x_ = dyn_params_->gyro_low_pass_ratio * filtered_gyro_x_ +
                       (1.F - dyn_params_->gyro_low_pass_ratio) * walk_core_state.angle_roll;
    filtered_gyro_y_ = dyn_params_->gyro_low_pass_ratio * filtered_gyro_y_ +
                       (1.F - dyn_params_->gyro_low_pass_ratio) * walk_core_state.angle_pitch;

    float balanced_adjustment =
      walk_core_state.walk_state == WalkState::STANDING
        ? 0.F
        : filtered_gyro_y_ *
            (filtered_gyro_y_ > 0
               ? dyn_params_->gyro_balance_factor.forward
               : dyn_params_->gyro_balance_factor.backward); // adjust ankle tilt in proportion to filtered_gyro_y_
    // joint_requests_.positions
    //   .values[walk_core_state.is_left_phase ? joint_indexes::R_ANKLE_PITCH : joint_indexes::L_ANKLE_PITCH] +=
    //   balanced_adjustment;
    joint_requests_.positions
      .values[walk_core_state.is_left_phase ? joint_indexes::R_HIP_PITCH : joint_indexes::L_HIP_PITCH] -=
      balanced_adjustment;
    // Lateral balance
    if (walk_core_state.walk_state == WalkState::STANDING) {
      balanced_adjustment = filtered_gyro_x_ * dyn_params_->gyro_balance_factor.sideways;
      joint_requests_.positions.values[joint_indexes::L_ANKLE_ROLL] += balanced_adjustment;
      joint_requests_.positions.values[joint_indexes::R_ANKLE_ROLL] += balanced_adjustment;
    }
  }

  void WalkGenerator::setStiffness() { // NOLINT(misc-unused-parameters)
    // int stiffness =
    //   walk_core_state.walk_state == WalkState::STANDING && (rclcpp::Clock{}.now() -
    //   walk_core_state.time_when_stand_began).seconds() > dyn_params_->stand_stiffness_delay
    //     ? 35 // Based on suggestion by Zichong we use just one value for stiffness for all joints for now.
    //     : walk_stiffness;
    for (uint8_t i = joint_indexes::L_HIP_YAW_PITCH; i < joint_indexes::R_ELBOW_YAW; ++i) {
      joint_requests_.stiffnesses.values[i] = dyn_params_->walk_stiffness;
    }
    joint_requests_.stiffnesses.values[joint_indexes::L_ANKLE_PITCH] = dyn_params_->ankle_pitch_stiffness;
    joint_requests_.stiffnesses.values[joint_indexes::L_ANKLE_ROLL] = dyn_params_->ankle_roll_stiffness;

    joint_requests_.stiffnesses.values[joint_indexes::R_ANKLE_PITCH] = dyn_params_->ankle_pitch_stiffness;
    joint_requests_.stiffnesses.values[joint_indexes::R_ANKLE_ROLL] = dyn_params_->ankle_roll_stiffness;
    // Head can setStiffnessmove freely
    joint_requests_.positions.values[joint_indexes::HEAD_PITCH] = 0.F;
    joint_requests_.positions.values[joint_indexes::HEAD_YAW] = 0.F;
    joint_requests_.joint_ignore[joint_indexes::HEAD_PITCH] = true;
    joint_requests_.joint_ignore[joint_indexes::HEAD_YAW] = true;
  }

  void WalkGenerator::updateLegJoints(const WalkCoreState& walk_core_state) {
    for (int i = 0; i < side::NUM_SIDES; ++i) {
      prev_foot_final_[i] = step_traj_[i].foot_final;
      step_traj_[i].foot_final = step_traj_[i].foot[FINAL_TRAJECTORY_TYPE];
    }

    if (dyn_params_->enable_gyro_compensation) {
      calcGyroOffset(walk_core_state);
      step_traj_[support_side_].foot_final.translation() -=
        Eigen::Vector3f(gyro_correction_states_[0].x(), gyro_correction_states_[0].y(), 0.F);
    }
  }

  void WalkGenerator::updateSwingControlPoint(const Twist2D& speed, float kick_power) {
    swing_control_point_.x() = 0.06F * kick_power;
    swing_control_point_.y() = 0.F;
    swing_control_point_.z() = 0.04F * kick_power + dyn_params_->base_foot_lift +
                               std::abs(speed.x) * dyn_params_->foot_lift_increase_factor.x +
                               std::abs(speed.y) * dyn_params_->foot_lift_increase_factor.y +
                               std::abs(speed.theta) * dyn_params_->foot_lift_increase_factor.theta;
    swing_target_offset_.x() = 0.04F * kick_power;
  }

  void WalkGenerator::generateStepTrajectory(const WalkCoreState& walk_core_state, const StepTraj::TrajType traj_type) {

    float t = std::clamp(walk_core_state.phase_time / walk_core_state.step_duration, 0.F, 1.F);
    float swing_height_scaler = 2.F;
    Eigen::Affine3f swing_target = walk_core_state.target_foot_poses[swing_side_];
    Eigen::Affine3f support_target = walk_core_state.target_foot_poses[support_side_];
    if (walk_core_state.walk_state == WalkState::STANDING || walk_core_state.walk_state == WalkState::STOPPING) {
      swing_height_scaler = 0.F;
    } else if (walk_core_state.walk_state == WalkState::STARTING) {
      swing_height_scaler = 2.F * dyn_params_->foot_lift_first_step_factor;
      swing_target = default_foot_poses_[swing_side_];
      support_target = default_foot_poses_[support_side_];
    }

    Eigen::Vector3f swing0 = step_traj_[swing_side_].foot[StepTraj::INITIAL_OFFSET].translation();
    Eigen::Vector3f swing2 = (swing_target.translation() + swing_control_point_ * swing_height_scaler);
    Eigen::Vector3f swing3 = swing_target.translation() + swing_target_offset_;

    Eigen::Vector3f support0 = step_traj_[support_side_].foot[StepTraj::INITIAL_OFFSET].translation();
    Eigen::Vector3f support3 = support_target.translation();

    step_traj_[swing_side_].foot[traj_type].translation() = bezierInterpolation(swing0, swing2, swing3, t);
    step_traj_[support_side_].foot[traj_type].translation() = support0 * (1 - t) + support3 * t;

    for (int i = 0; i < side::NUM_SIDES; ++i) {
      Eigen::Affine3f target_rotation = (i == swing_side_) ? swing_target : support_target;
      const Eigen::Quaternionf target_foot_rotation(target_rotation.rotation());
      const Eigen::Quaternionf initial_foot_rotation(step_traj_[i].foot[StepTraj::INITIAL_OFFSET].rotation());

      Eigen::Quaternionf interpolated_quaternion = initial_foot_rotation.slerp(t, target_foot_rotation);
      step_traj_[i].foot[traj_type].linear() = interpolated_quaternion.toRotationMatrix();
    }
  }

} // namespace nomadz_motion_control
