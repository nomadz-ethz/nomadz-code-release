#pragma once

#include <Eigen/Geometry>

#include "nomadz_kinematics/types.hpp"

/**
 * Forward kinematics for the NAO robot.
 *
 * The functions are named according to the frame convention used in the NAO URDF,
 * which uses the DH convention i.e. only 1 DOF per transformation.

 * NOTE - nomenclature:
 * If we have Frame Alpha as starting Frame and Frame Beta as target Frame -> Function name: BetaFromAlpha
 * And for the variables in the function: transform_beta_alpha
 */

namespace nomadz_kinematics {

  // head chain
  Eigen::Affine3f torsoFromNeck(float head_yaw);
  Eigen::Affine3f neckFromHead(float head_pitch);
  Eigen::Affine3f torsoFromHead(float head_yaw, float head_pitch);

  // arm chain
  Eigen::Affine3f torsoFromShoulder(bool left, float shoulder_pitch);
  Eigen::Affine3f shoulderFromBicep(float shoulder_roll);
  Eigen::Affine3f bicepFromElbow(bool left, float elbow_yaw);
  Eigen::Affine3f elbowFromForeArm(float elbow_roll);
  Eigen::Affine3f foreArmFromHand(float wrist_yaw);
  Eigen::Affine3f handFromFinger();
  Eigen::Affine3f
  torsoFromFinger(bool left, float shoulder_pitch, float shoulder_roll, float elbow_yaw, float elbow_roll, float wrist_yaw);

  // leg chain
  Eigen::Affine3f torsoFromPelvis(bool left, float hip_yaw_pitch);
  Eigen::Affine3f pelvisFromHip(float hip_roll);
  Eigen::Affine3f hipFromThigh(float hip_pitch);
  Eigen::Affine3f thighFromTibia(float knee_pitch);
  Eigen::Affine3f tibiaFromAnkle(float ankle_pitch);
  Eigen::Affine3f ankleFromFoot(float ankle_roll);
  Eigen::Affine3f torsoFromFoot(
    bool left, float hip_yaw_pitch, float hip_roll, float hip_pitch, float knee_pitch, float ankle_pitch, float ankle_roll);

  // limb transform chains
  void calculateArmChain(bool left, const JointAngles& joint_positions, LimbTransforms& limb_tfs);
  void calculateLegChain(bool left, const JointAngles& joint_positions, LimbTransforms& limb_tfs);
  void calculateHeadChain(const JointAngles& joint_positions, LimbTransforms& limb_tfs);

  // all limbs to torso
  void calculateTorsoFromArmLimbs(bool left, const JointAngles& joint_positions, LimbTransforms& limb_tfs);
  void calculateTorsoFromLegLimbs(bool left, const JointAngles& joint_positions, LimbTransforms& limb_tfs);
  void calculateTorsoFromHeadLimbs(const JointAngles& joint_positions, LimbTransforms& limb_tfs);

} // namespace nomadz_kinematics
