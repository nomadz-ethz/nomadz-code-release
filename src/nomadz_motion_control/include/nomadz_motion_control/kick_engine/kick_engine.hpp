#pragma once

#include <rclcpp/rclcpp.hpp>

#include "nomadz_core/math/constants.hpp"
#include "nomadz_kinematics/inverse_kinematics.hpp"
#include "nomadz_kinematics/robot_model.hpp"
#include "nomadz_motion_control/interpolation.hpp"
#include "nomadz_motion_control/motion_base.hpp"
#include "nomadz_motion_control/kick_engine/kick_trajectory_loader.hpp"

namespace nomadz_motion_control {
  class KickEngine : public MotionBase {
    using RobotModel = nomadz_kinematics::RobotModel;

  public:
    explicit KickEngine(const rclcpp::NodeOptions& options);

    bool kickExecuted() const;
    nao_lola_command_msgs::msg::JointRequests getJointRequest(Eigen::Affine3f pose) const;
    nao_lola_command_msgs::msg::JointRequests current_request_msg; // stores the current to-be published joint message
    Eigen::Affine3f current_kick_foot_pose;
    Eigen::Affine3f current_support_foot_pose;
    std::shared_ptr<RobotModel> robot_model;

    // *** REMOVE
    void publicReset() {
      phase_time_ = 0.0F;
      kick_phase_ptr_ = kick_trajectories_[kick_id_].kick_phases.cbegin();
      current_kick_foot_pose = kick_trajectories_[kick_id_].starting_position;
    }

  private:
    void updateJointRequests() override;
    void updateMotionInfoMsg() override;
    bool isLeavingPossible() override;
    void reset() override;
    void updateKickFootPose(float ratio);
    std::vector<kick_engine::KickTrajectory> kick_trajectories_;

    uint kick_id_ = 0;                                                   // kick type identifier
    float phase_time_ = 0.0F;                                            // record time of the current kick phase
    bool left_ = true;                                                   // class member determine if left leg is kicking
    std::vector<kick_engine::KickPhase>::const_iterator kick_phase_ptr_; // points to the current active kick phase
    constexpr static const float STIFFNESS = 0.7F;                       // TODO(chang): tune this value
  };
} // namespace nomadz_motion_control
