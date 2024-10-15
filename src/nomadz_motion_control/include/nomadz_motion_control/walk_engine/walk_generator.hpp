#pragma once

#include <Eigen/Geometry>

#include "nomadz_definitions/limbs.hpp"
#include "nomadz_core/geometry/twist.hpp"

#include "nomadz_motion_control/joint_requests.hpp"
#include "nomadz_motion_control/walk_engine/walk_core_state.hpp"
#include "nomadz_motion_control/walk_engine/walk_generator_data.hpp"
#include "walk_engine_parameters.hpp"

namespace nomadz_motion_control {
  class WalkGenerator {
    using Side = nomadz_definitions::side::Side;

  public:
    explicit WalkGenerator(const std::shared_ptr<walk_engine_parameters::Params> dyn_params);

    void generateJointRequests(const WalkCoreState& walk_core_state);

    WalkGeneratorData getWalkGeneratorData() const { return walk_generator_data_; };

    void updateSwingControlPoint(const nomadz_core::Twist2D& speed, float kick_power = 0.F);

  private:
    void updateFromCoreState(const WalkCoreState& walk_core_state);

    void generateStepTrajectory(const WalkCoreState& walk_core_state, StepTraj::TrajType traj_type);

    nomadz_core::Twist2D calcOdometryOffset(const WalkCoreState& walk_core_state);

    void updateLegJoints(const WalkCoreState& walk_core_state);

    void calcGyroOffset(const WalkCoreState& walk_core_state);

    void setStiffness();

    void applyGyroBalance(const WalkCoreState& walk_core_state);

    static constexpr StepTraj::TrajType FINAL_TRAJECTORY_TYPE = StepTraj::CURRENT_OFFSET;

    JointRequests joint_requests_;
    const std::shared_ptr<walk_engine_parameters::Params> dyn_params_;

    Eigen::Vector3f swing_control_point_{0.F, 0.F, 0.F};
    Eigen::Vector3f swing_target_offset_{0.F, 0.F, 0.F};
    StepTraj step_traj_[nomadz_definitions::side::NUM_SIDES]; /**< The trajectory of the current step. */
    Eigen::Affine3f default_foot_poses_[nomadz_definitions::side::NUM_SIDES];
    Side swing_side_{Side::RIGHT};
    Side support_side_{Side::LEFT};
    Eigen::Affine2f step_correction_;
    Eigen::Vector2f gyro_correction_states_[3];
    float filtered_gyro_x_;
    float filtered_gyro_y_;
    Eigen::Affine3f prev_foot_final_[nomadz_definitions::side::NUM_SIDES];
    Eigen::Affine3f augmented_foot_final_[nomadz_definitions::side::NUM_SIDES];
    std::array<Eigen::Affine3f, nomadz_definitions::side::NUM_SIDES> prev_measured_foot_poses_;
    std::array<Eigen::Affine3f, nomadz_definitions::side::NUM_SIDES> prev_target_foot_poses_;

    WalkGeneratorData walk_generator_data_;
  };
} // namespace nomadz_motion_control
