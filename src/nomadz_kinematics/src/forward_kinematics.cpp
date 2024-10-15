#include "nomadz_kinematics/forward_kinematics.hpp"

#include <cmath>

#include <Eigen/Geometry>

#include "nomadz_definitions/joint_indexes.hpp"

#include "nomadz_kinematics/robot_dimensions.hpp"

using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Translation3f;
using Eigen::Vector3f;

namespace limbs = nomadz_definitions::limbs;
namespace joint_indexes = nomadz_definitions::joint_indexes;

namespace nomadz_kinematics {

  // head chain
  Affine3f torsoFromNeck(float head_yaw) {
    Affine3f transform_torso_neck = Translation3f(0.0, 0.0, NECK_OFFSET_Z) * AngleAxisf(head_yaw, Vector3f::UnitZ());
    return transform_torso_neck;
  }

  Affine3f neckFromHead(float head_pitch) {
    Affine3f transform_neck_head;
    transform_neck_head = AngleAxisf(head_pitch, Vector3f::UnitY());
    return transform_neck_head;
  }

  Affine3f torsoFromHead(float head_yaw, float head_pitch) {
    Affine3f transform_torso_head;
    transform_torso_head = torsoFromNeck(head_yaw) * neckFromHead(head_pitch);
    return transform_torso_head;
  }

  // arm chain
  Affine3f torsoFromShoulder(bool left, float shoulder_pitch) {
    float const left_sign = left ? 1.0F : -1.0F;
    Affine3f transform_torso_shoulder =
      Translation3f(0.0, left_sign * SHOULDER_OFFSET_Y, SHOULDER_OFFSET_Z) * AngleAxisf(shoulder_pitch, Vector3f::UnitY());
    return transform_torso_shoulder;
  }

  Affine3f shoulderFromBicep(float shoulder_roll) {
    Affine3f transform_shoulder_bicep;
    transform_shoulder_bicep = AngleAxisf(shoulder_roll, Vector3f::UnitZ());
    return transform_shoulder_bicep;
  }

  Affine3f bicepFromElbow(bool left, float elbow_yaw) {
    float const left_sign = left ? 1.0F : -1.0F;
    Affine3f transform_bicep_elbow =
      Translation3f(UPPER_ARM_LENGTH, left_sign * ELBOW_OFFSET_Y, 0.F) * AngleAxisf(elbow_yaw, Vector3f::UnitX());
    return transform_bicep_elbow;
  }

  Affine3f elbowFromForeArm(float elbow_roll) {
    Affine3f transform_elbow_forearm;
    transform_elbow_forearm = AngleAxisf(elbow_roll, Vector3f::UnitX());
    return transform_elbow_forearm;
  }

  Affine3f foreArmFromHand(float wrist_yaw) {
    Affine3f transform_forearm_hand = Translation3f(LOWER_ARM_LENGTH, 0.F, 0.F) * AngleAxisf(wrist_yaw, Vector3f::UnitX());
    return transform_forearm_hand;
  }

  Affine3f handFromFinger() {
    Affine3f transform_hand_finger = Translation3f(HAND_OFFSET_X, 0.F, -HAND_OFFSET_Z) * AngleAxisf::Identity();
    return transform_hand_finger;
  }

  Affine3f
  torsoFromFinger(bool left, float shoulder_pitch, float shoulder_roll, float elbow_yaw, float elbow_roll, float wrist_yaw) {
    Affine3f transform_torso_finger;
    transform_torso_finger = torsoFromShoulder(left, shoulder_pitch) * shoulderFromBicep(shoulder_roll) *
                             bicepFromElbow(left, elbow_yaw) * elbowFromForeArm(elbow_roll) * foreArmFromHand(wrist_yaw) *
                             handFromFinger();
    return transform_torso_finger;
  }

  // leg chain
  Affine3f torsoFromPelvis(bool left, float hip_yaw_pitch) {
    float const left_sign = left ? 1.0F : -1.0F;
    Affine3f transform_torso_pelvis =
      Translation3f(0.0, left_sign * HIP_OFFSET_Y, -HIP_OFFSET_Z) *
      AngleAxisf(hip_yaw_pitch, Vector3f(0.F, std::sqrt(2.F) / 2.F, -left_sign * std::sqrt(2.F) / 2.F));
    return transform_torso_pelvis;
  }

  Affine3f pelvisFromHip(float hip_roll) {
    Affine3f transform_pelvis_hip;
    transform_pelvis_hip = AngleAxisf(hip_roll, Vector3f(1.F, 0.F, 0.F));
    return transform_pelvis_hip;
  }
  Affine3f hipFromThigh(float hip_pitch) {
    Affine3f transform_hip_thigh;
    transform_hip_thigh = AngleAxisf(hip_pitch, Vector3f::UnitY());
    return transform_hip_thigh;
  }

  Affine3f thighFromTibia(float knee_pitch) {
    Affine3f transform_thigh_tibia = Translation3f(0.F, 0.F, -THIGH_LENGTH) * AngleAxisf(knee_pitch, Vector3f::UnitY());
    return transform_thigh_tibia;
  }
  Affine3f tibiaFromAnkle(float ankle_pitch) {
    Affine3f transform_tibia_ankle = Translation3f(0.F, 0.F, -TIBIA_LENGTH) * AngleAxisf(ankle_pitch, Vector3f::UnitY());
    return transform_tibia_ankle;
  }
  Affine3f ankleFromFoot(float ankle_roll) {
    Affine3f transform_ankle_foot;
    transform_ankle_foot = AngleAxisf(ankle_roll, Vector3f::UnitX());
    return transform_ankle_foot;
  }

  Affine3f torsoFromFoot(
    bool left, float hip_yaw_pitch, float hip_roll, float hip_pitch, float knee_pitch, float ankle_pitch, float ankle_roll) {
    Affine3f transform_torso_foot;
    transform_torso_foot = torsoFromPelvis(left, hip_yaw_pitch) * pelvisFromHip(hip_roll) * hipFromThigh(hip_pitch) *
                           thighFromTibia(knee_pitch) * tibiaFromAnkle(ankle_pitch) * ankleFromFoot(ankle_roll);
    return transform_torso_foot;
  }

  // limb transform chains
  void calculateArmChain(bool left, const JointAngles& joint_positions, LimbTransforms& limb_tfs) {
    int joint_index = left ? joint_indexes::L_SHOULDER_PITCH : joint_indexes::R_SHOULDER_PITCH;
    int limb_index = left ? limbs::SHOULDER_LEFT : limbs::SHOULDER_RIGHT;

    limb_tfs[limb_index] = torsoFromShoulder(left, joint_positions[joint_index]);
    limb_tfs[limb_index + 1] = shoulderFromBicep(joint_positions[joint_index + 1]);
    limb_tfs[limb_index + 2] = bicepFromElbow(left, joint_positions[joint_index + 2]);
    limb_tfs[limb_index + 3] = elbowFromForeArm(joint_positions[joint_index + 3]);
    limb_tfs[limb_index + 4] = foreArmFromHand(joint_positions[joint_index + 4]);
  }

  void calculateLegChain(bool left, const JointAngles& joint_positions, LimbTransforms& limb_tfs) {
    int limb_index = left ? limbs::PELVIS_LEFT : limbs::PELVIS_RIGHT;
    limb_tfs[limb_index] = torsoFromPelvis(left, joint_positions[joint_indexes::L_HIP_YAW_PITCH]);

    int joint_index = left ? joint_indexes::L_HIP_ROLL : joint_indexes::R_HIP_ROLL;
    limb_tfs[limb_index + 1] = pelvisFromHip(joint_positions[joint_index]);
    limb_tfs[limb_index + 2] = hipFromThigh(joint_positions[joint_index + 1]);
    limb_tfs[limb_index + 3] = thighFromTibia(joint_positions[joint_index + 2]);
    limb_tfs[limb_index + 4] = tibiaFromAnkle(joint_positions[joint_index + 3]);
    limb_tfs[limb_index + 5] = ankleFromFoot(joint_positions[joint_index + 4]);
  }

  void calculateHeadChain(const JointAngles& joint_positions, LimbTransforms& limb_tfs) {
    const float head_yaw = joint_positions[joint_indexes::HEAD_YAW];
    const float head_pitch = joint_positions[joint_indexes::HEAD_PITCH];

    limb_tfs[limbs::NECK] = torsoFromNeck(head_yaw);
    limb_tfs[limbs::HEAD] = neckFromHead(head_pitch);
  }

  // all limbs to torso
  void calculateTorsoFromArmLimbs(bool left, const JointAngles& joint_positions, LimbTransforms& limb_tfs) {
    int joint_index = left ? joint_indexes::L_SHOULDER_PITCH : joint_indexes::R_SHOULDER_PITCH;
    int limb_index = left ? limbs::SHOULDER_LEFT : limbs::SHOULDER_RIGHT;

    limb_tfs[limb_index] = torsoFromShoulder(left, joint_positions[joint_index]);
    limb_tfs[limb_index + 1] = limb_tfs[limb_index] * shoulderFromBicep(joint_positions[joint_index + 1]);
    limb_tfs[limb_index + 2] = limb_tfs[limb_index + 1] * bicepFromElbow(left, joint_positions[joint_index + 2]);
    limb_tfs[limb_index + 3] = limb_tfs[limb_index + 2] * elbowFromForeArm(joint_positions[joint_index + 3]);
    limb_tfs[limb_index + 4] = limb_tfs[limb_index + 3] * foreArmFromHand(joint_positions[joint_index + 4]);
  }

  void calculateTorsoFromLegLimbs(bool left, const JointAngles& joint_positions, LimbTransforms& limb_tfs) {
    int limb_index = left ? limbs::PELVIS_LEFT : limbs::PELVIS_RIGHT;
    limb_tfs[limb_index] = torsoFromPelvis(left, joint_positions[joint_indexes::L_HIP_YAW_PITCH]);

    int joint_index = left ? joint_indexes::L_HIP_ROLL : joint_indexes::R_HIP_ROLL;
    limb_tfs[limb_index + 1] = limb_tfs[limb_index] * pelvisFromHip(joint_positions[joint_index]);
    limb_tfs[limb_index + 2] = limb_tfs[limb_index + 1] * hipFromThigh(joint_positions[joint_index + 1]);
    limb_tfs[limb_index + 3] = limb_tfs[limb_index + 2] * thighFromTibia(joint_positions[joint_index + 2]);
    limb_tfs[limb_index + 4] = limb_tfs[limb_index + 3] * tibiaFromAnkle(joint_positions[joint_index + 3]);
    limb_tfs[limb_index + 5] = limb_tfs[limb_index + 4] * ankleFromFoot(joint_positions[joint_index + 4]);
  }

  void calculateTorsoFromHeadLimbs(const JointAngles& joint_positions, LimbTransforms& limb_tfs) {
    const float head_yaw = joint_positions[joint_indexes::HEAD_YAW];
    const float head_pitch = joint_positions[joint_indexes::HEAD_PITCH];

    limb_tfs[limbs::NECK] = torsoFromNeck(head_yaw);
    limb_tfs[limbs::HEAD] = limb_tfs[limbs::NECK] * neckFromHead(head_pitch);
  }

} // namespace nomadz_kinematics
