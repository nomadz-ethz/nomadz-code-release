#include "nomadz_motion_control/kick_engine/kick_engine.hpp"

using nomadz_core::constants::MOTION_CYCLE_TIME;
using nomadz_motion_control::bezierInterpolation;
using nomadz_motion_control::linearInterpolation;
using JointIndexes = nomadz_definitions::joint_indexes::JointIndexes;

namespace nomadz_motion_control {
  KickEngine::KickEngine(const rclcpp::NodeOptions& options)
      : MotionBase("kick_engine", options), robot_model(std::make_shared<RobotModel>()) {
    kick_trajectories_ = kick_engine::loadKickTrajectoriesFromYaml();
    kick_phase_ptr_ = kick_trajectories_[kick_id_].kick_phases.cbegin();
    current_kick_foot_pose = kick_trajectories_[kick_id_].starting_position;
  }

  // only returns true if the kick is fully executed
  bool KickEngine::kickExecuted() const {
    return kick_phase_ptr_ == kick_trajectories_[kick_id_].kick_phases.end();
  }

  void KickEngine::updateJointRequests() {
    assert(kick_phase_ptr_ != kick_trajectories_[kick_id_].kick_phases.end());
    assert(phase_time_ < kick_phase_ptr_->duration + MOTION_CYCLE_TIME);

    const float ratio = std::min(phase_time_ / kick_phase_ptr_->duration, 1.0F);
    updateKickFootPose(ratio);

    const Eigen::Affine3f shifted_ee_pose = left_ ? current_kick_foot_pose * Eigen::Translation3f{0.0, 0.05, -0.23}
                                                  : current_kick_foot_pose * Eigen::Translation3f{0.0, -0.05, -0.23};

    // generate the corresponding joint request msg
    current_request_msg = getJointRequest(shifted_ee_pose);

    if (ratio == 1.0F) {
      kick_phase_ptr_++;
      phase_time_ = 0.0F;
    } else {
      phase_time_ += MOTION_CYCLE_TIME;
    }

    // update the field
    current_joint_requests_ = JointRequests(current_request_msg);
  }

  void KickEngine::updateMotionInfoMsg() {
    // TODO(chang): implement this interface
    motion_info_msg_.is_leaving_possible = kickExecuted();
    motion_info_msg_.is_motion_done = kickExecuted();
    motion_info_msg_.odometry_offset = geometry_msgs::msg::Twist();
    motion_info_msg_.executed_motion_request = current_motion_request_msg_;
  }

  bool KickEngine::isLeavingPossible() {
    bool is_leaving_possible = kickExecuted();
    if (is_leaving_possible) {
      request_leave_ = false;
    }
    return is_leaving_possible;
  }
  void KickEngine::reset() {
    phase_time_ = 0.0F;
    kick_phase_ptr_ = kick_trajectories_[kick_id_].kick_phases.cbegin();
    current_kick_foot_pose = kick_trajectories_[kick_id_].starting_position;
    request_leave_ = false;
  }

  nao_lola_command_msgs::msg::JointRequests KickEngine::getJointRequest(Eigen::Affine3f pose) const {
    // computes the joint angles for the given EE pose
    std::array<float, 6> joint_angles;
    nomadz_kinematics::inverseTorsoFromFoot(
      left_, pose, joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5]);

    // generate the corresponding joint request msg
    nao_lola_command_msgs::msg::JointRequests joint_msg{};
    for (int i = 0; i < 6; i++) {
      if (left_ || i == 0) {
        joint_msg.indexes.push_back(JointIndexes::L_HIP_YAW_PITCH + i);
      } else {
        joint_msg.indexes.push_back(JointIndexes::L_HIP_YAW_PITCH + 5 + i);
      }
      joint_msg.positions.push_back(joint_angles[i]);
      joint_msg.stiffnesses.push_back(STIFFNESS);
    }
    return joint_msg;
  }

  void KickEngine::updateKickFootPose(float ratio) {
    switch (kick_phase_ptr_->interpolation_type) {
    case InterpolationType::LINEAR:
      current_kick_foot_pose = linearInterpolation(current_kick_foot_pose, kick_phase_ptr_->key_frames.at(0), ratio);
      break;
    case InterpolationType::QUADRATIC_BEZIER:
      current_kick_foot_pose = bezierInterpolation(
        current_kick_foot_pose, kick_phase_ptr_->key_frames.at(0), kick_phase_ptr_->key_frames.at(1), ratio);
      break;
    case InterpolationType::CUBIC_BEZIER:
      current_kick_foot_pose = bezierInterpolation(current_kick_foot_pose,
                                                   kick_phase_ptr_->key_frames.at(0),
                                                   kick_phase_ptr_->key_frames.at(1),
                                                   kick_phase_ptr_->key_frames.at(2),
                                                   ratio);
      break;
    default:
      throw std::invalid_argument("interpolation type invalid");
    }
  }

} // namespace nomadz_motion_control
