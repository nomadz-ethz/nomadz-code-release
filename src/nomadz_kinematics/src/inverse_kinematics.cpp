#include "nomadz_kinematics/inverse_kinematics.hpp"

#include <cmath>

#include "nomadz_definitions/joint_indexes.hpp"
#include "nomadz_kinematics/forward_kinematics.hpp"

using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Translation3f;
using Eigen::Vector3f;

namespace joint_indexes = nomadz_definitions::joint_indexes;

namespace nomadz_kinematics {
  float normalizeAngle(float angle) {
    if (angle > M_PI / 2.F) {
      angle -= M_PI;
    } else if (angle < -M_PI / 2.F) {
      angle += M_PI;
    }
    return angle;
  }

  void inverseTorsoFromHead(const Affine3f& target_pose, float& head_yaw, float& head_pitch) {
    // Determine the head_yaw angle using the Yaw orientation
    Vector3f rotation_c1 = target_pose.rotation().col(0);
    head_yaw = std::atan2(rotation_c1.y(), rotation_c1.x());
    // Determine the head_pitch angle using the Pitch orientation
    head_pitch = std::atan2(-rotation_c1.z(), std::sqrt(std::pow(rotation_c1.x(), 2.F) + std::pow(rotation_c1.y(), 2.F)));
  }

  void inverseTorsoFromFoot(bool left,
                            const Affine3f& target_pose,
                            float& hip_yaw_pitch,
                            float& hip_roll,
                            float& hip_pitch,
                            float& knee_pitch,
                            float& ankle_pitch,
                            float& ankle_roll) {
    // Determine the hip_yaw_pitch angle using the Yaw orientation. Do it only for the left side as the joints are
    // kinematically coupled

    // This is some geometry magic. It is calculated in close-form using the Co-plane alignment between the x-axis of the
    // target rotation and the x-axis of the pelvis orientation.
    Vector3f rotation_c1 = target_pose.rotation().col(0);
    const float mirror_factor = left ? 1.F : -1.F;
    if (mirror_factor * rotation_c1.y() == rotation_c1.z()) {
      // Special case: both axis are parallel
      hip_yaw_pitch = -mirror_factor * std::atan2(std::sqrt(2.F) * rotation_c1.y(), rotation_c1.x());
    } else {
      // General case: both axis are in the same plane but not parallel
      Vector3f translation = target_pose.translation() - Vector3f(0.0F, left ? HIP_OFFSET_Y : -HIP_OFFSET_Y, -HIP_OFFSET_Z);
      const float b =
        (mirror_factor * translation.z() - translation.y()) / (rotation_c1.y() - mirror_factor * rotation_c1.z());
      hip_yaw_pitch = std::atan2(-mirror_factor * std::sqrt(2.F) * (b * rotation_c1.y() + translation.y()),
                                 (b * rotation_c1.x() + translation.x()));
    }
    // There are always two solutions.
    hip_yaw_pitch = normalizeAngle(hip_yaw_pitch);

    // Compute the target pose in pelvis frame
    Affine3f transform_pelvis_foot_target = torsoFromPelvis(left, hip_yaw_pitch).inverse() * target_pose;
    hip_roll = std::atan2(transform_pelvis_foot_target.translation().y(), -transform_pelvis_foot_target.translation().z());
    Affine3f transform_hip_foot_target = pelvisFromHip(hip_roll).inverse() * transform_pelvis_foot_target;
    float c;
    float hp_additional;
    if (transform_hip_foot_target.translation().norm() >= THIGH_LENGTH + TIBIA_LENGTH) {
      knee_pitch = 0.F;
      c = THIGH_LENGTH + TIBIA_LENGTH;
      hp_additional = 0.F;
    } else {
      c = transform_hip_foot_target.translation().norm();
      // the HipRoll can already be determined using the translation of the target pose in pelvis frame
      // Compute the target pose in hip frame
      // Now the KneePitch can be determined using the law of cosines
      knee_pitch = M_PI - std::acos((std::pow(THIGH_LENGTH, 2.F) + std::pow(TIBIA_LENGTH, 2.F) - std::pow(c, 2.F)) /
                                    (2.F * THIGH_LENGTH * TIBIA_LENGTH));
      hp_additional =
        std::acos((std::pow(THIGH_LENGTH, 2.F) + std::pow(c, 2.F) - std::pow(TIBIA_LENGTH, 2.F)) / (2.F * THIGH_LENGTH * c));
    }

    // The HipPitch can be determined using the translation of the target pose in hip frame
    hip_pitch =
      -std::atan2(transform_hip_foot_target.translation().x(), -transform_hip_foot_target.translation().z()) - hp_additional;

    // Use the remain Target rotation to determine the ankle_pitch and ankle_roll
    ankle_pitch =
      std::atan2(-transform_hip_foot_target.rotation().coeff(2, 0), transform_hip_foot_target.rotation().coeff(0, 0)) -
      knee_pitch - hip_pitch;
    auto angle_to_foot_rotation =
      AngleAxisf(-(hip_pitch + knee_pitch + ankle_pitch), Vector3f::UnitY()) * transform_hip_foot_target.rotation();
    ankle_roll = std::atan2(angle_to_foot_rotation.coeff(2, 1), angle_to_foot_rotation.coeff(1, 1));
  }

  void inverseTorsoFromFoot(const Affine3f& leftTarget,
                            const Affine3f& rightTarget,
                            std::array<float, NUM_DOF>& current_joint_position) {
    inverseTorsoFromFoot(true,
                         leftTarget,
                         current_joint_position[joint_indexes::L_HIP_YAW_PITCH],
                         current_joint_position[joint_indexes::L_HIP_ROLL],
                         current_joint_position[joint_indexes::L_HIP_PITCH],
                         current_joint_position[joint_indexes::L_KNEE_PITCH],
                         current_joint_position[joint_indexes::L_ANKLE_PITCH],
                         current_joint_position[joint_indexes::L_ANKLE_ROLL]);
    inverseTorsoFromFoot(false,
                         rightTarget,
                         current_joint_position[joint_indexes::L_HIP_YAW_PITCH],
                         current_joint_position[joint_indexes::R_HIP_ROLL],
                         current_joint_position[joint_indexes::R_HIP_PITCH],
                         current_joint_position[joint_indexes::R_KNEE_PITCH],
                         current_joint_position[joint_indexes::R_ANKLE_PITCH],
                         current_joint_position[joint_indexes::R_ANKLE_ROLL]);
  }

} // namespace nomadz_kinematics
