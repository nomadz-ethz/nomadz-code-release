#pragma once

#include <map>
#include <array>
#include <algorithm>

namespace nomadz_definitions {
  namespace legacy_joint_indexes {
    enum LegacyJointIndexes {
      HEAD_YAW = 0,
      HEAD_PITCH,
      L_SHOULDER_PITCH,
      L_SHOULDER_ROLL,
      L_ELBOW_YAW,
      L_ELBOW_ROLL,
      L_WRIST_YAW,
      L_HAND,
      R_SHOULDER_PITCH,
      R_SHOULDER_ROLL,
      R_ELBOW_YAW,
      R_ELBOW_ROLL,
      R_WRIST_YAW,
      R_HAND,
      L_HIP_YAW_PITCH,
      L_HIP_ROLL,
      L_HIP_PITCH,
      L_KNEE_PITCH,
      L_ANKLE_PITCH,
      L_ANKLE_ROLL,
      R_HIP_YAW_PITCH,
      R_HIP_ROLL,
      R_HIP_PITCH,
      R_KNEE_PITCH,
      R_ANKLE_PITCH,
      R_ANKLE_ROLL,
      NUM_JOINTS
    };
  } // namespace legacy_joint_indexes

  namespace joint_indexes {
    enum JointIndexes {
      HEAD_YAW = 0,
      HEAD_PITCH,
      L_SHOULDER_PITCH = 2,
      L_SHOULDER_ROLL,
      L_ELBOW_YAW,
      L_ELBOW_ROLL,
      L_WRIST_YAW,
      L_HIP_YAW_PITCH = 7,
      L_HIP_ROLL,
      L_HIP_PITCH,
      L_KNEE_PITCH,
      L_ANKLE_PITCH,
      L_ANKLE_ROLL,
      R_HIP_ROLL = 13,
      R_HIP_PITCH,
      R_KNEE_PITCH,
      R_ANKLE_PITCH,
      R_ANKLE_ROLL,
      R_SHOULDER_PITCH = 18,
      R_SHOULDER_ROLL,
      R_ELBOW_YAW,
      R_ELBOW_ROLL,
      R_WRIST_YAW,
      L_HAND = 23,
      R_HAND,
      NUM_JOINTS
    };
  } // namespace joint_indexes

  inline const std::array<joint_indexes::JointIndexes, 10> PITCH_JOINT_NAMES = {joint_indexes::HEAD_PITCH,
                                                                                joint_indexes::L_SHOULDER_PITCH,
                                                                                joint_indexes::R_SHOULDER_PITCH,
                                                                                joint_indexes::L_HIP_YAW_PITCH,
                                                                                joint_indexes::L_HIP_PITCH,
                                                                                joint_indexes::L_KNEE_PITCH,
                                                                                joint_indexes::L_ANKLE_PITCH,
                                                                                joint_indexes::R_HIP_PITCH,
                                                                                joint_indexes::R_KNEE_PITCH,
                                                                                joint_indexes::R_ANKLE_PITCH};

  inline const std::map<joint_indexes::JointIndexes, joint_indexes::JointIndexes> JOINT_NAME_MIRROR_MAP = {
    {joint_indexes::HEAD_YAW, joint_indexes::HEAD_YAW},
    {joint_indexes::HEAD_PITCH, joint_indexes::HEAD_PITCH},
    {joint_indexes::L_SHOULDER_PITCH, joint_indexes::R_SHOULDER_PITCH},
    {joint_indexes::L_SHOULDER_ROLL, joint_indexes::R_SHOULDER_ROLL},
    {joint_indexes::L_ELBOW_YAW, joint_indexes::R_ELBOW_YAW},
    {joint_indexes::L_ELBOW_ROLL, joint_indexes::R_ELBOW_ROLL},
    {joint_indexes::L_WRIST_YAW, joint_indexes::R_WRIST_YAW},
    {joint_indexes::L_HIP_YAW_PITCH, joint_indexes::L_HIP_YAW_PITCH},
    {joint_indexes::L_HIP_ROLL, joint_indexes::R_HIP_ROLL},
    {joint_indexes::L_HIP_PITCH, joint_indexes::R_HIP_PITCH},
    {joint_indexes::L_KNEE_PITCH, joint_indexes::R_KNEE_PITCH},
    {joint_indexes::L_ANKLE_PITCH, joint_indexes::R_ANKLE_PITCH},
    {joint_indexes::L_ANKLE_ROLL, joint_indexes::R_ANKLE_ROLL},
    {joint_indexes::R_HIP_ROLL, joint_indexes::L_HIP_ROLL},
    {joint_indexes::R_HIP_PITCH, joint_indexes::L_HIP_PITCH},
    {joint_indexes::R_KNEE_PITCH, joint_indexes::L_KNEE_PITCH},
    {joint_indexes::R_ANKLE_PITCH, joint_indexes::L_ANKLE_PITCH},
    {joint_indexes::R_ANKLE_ROLL, joint_indexes::L_ANKLE_ROLL},
    {joint_indexes::R_SHOULDER_PITCH, joint_indexes::L_SHOULDER_PITCH},
    {joint_indexes::R_SHOULDER_ROLL, joint_indexes::L_SHOULDER_ROLL},
    {joint_indexes::R_ELBOW_YAW, joint_indexes::L_ELBOW_YAW},
    {joint_indexes::R_ELBOW_ROLL, joint_indexes::L_ELBOW_ROLL},
    {joint_indexes::R_WRIST_YAW, joint_indexes::L_WRIST_YAW},
    {joint_indexes::L_HAND, joint_indexes::R_HAND},
    {joint_indexes::R_HAND, joint_indexes::L_HAND}};

  inline const std::map<joint_indexes::JointIndexes, legacy_joint_indexes::LegacyJointIndexes> JOINT_NAMES_TO_LEGACY_MAP = {
    {joint_indexes::HEAD_YAW, legacy_joint_indexes::HEAD_YAW},
    {joint_indexes::HEAD_PITCH, legacy_joint_indexes::HEAD_PITCH},
    {joint_indexes::L_SHOULDER_PITCH, legacy_joint_indexes::L_SHOULDER_PITCH},
    {joint_indexes::L_SHOULDER_ROLL, legacy_joint_indexes::L_SHOULDER_ROLL},
    {joint_indexes::L_ELBOW_YAW, legacy_joint_indexes::L_ELBOW_YAW},
    {joint_indexes::L_ELBOW_ROLL, legacy_joint_indexes::L_ELBOW_ROLL},
    {joint_indexes::L_WRIST_YAW, legacy_joint_indexes::L_WRIST_YAW},
    {joint_indexes::L_HIP_YAW_PITCH, legacy_joint_indexes::L_HIP_YAW_PITCH},
    {joint_indexes::L_HIP_ROLL, legacy_joint_indexes::L_HIP_ROLL},
    {joint_indexes::L_HIP_PITCH, legacy_joint_indexes::L_HIP_PITCH},
    {joint_indexes::L_KNEE_PITCH, legacy_joint_indexes::L_KNEE_PITCH},
    {joint_indexes::L_ANKLE_PITCH, legacy_joint_indexes::L_ANKLE_PITCH},
    {joint_indexes::L_ANKLE_ROLL, legacy_joint_indexes::L_ANKLE_ROLL},
    {joint_indexes::R_HIP_ROLL, legacy_joint_indexes::R_HIP_ROLL},
    {joint_indexes::R_HIP_PITCH, legacy_joint_indexes::R_HIP_PITCH},
    {joint_indexes::R_KNEE_PITCH, legacy_joint_indexes::R_KNEE_PITCH},
    {joint_indexes::R_ANKLE_PITCH, legacy_joint_indexes::R_ANKLE_PITCH},
    {joint_indexes::R_ANKLE_ROLL, legacy_joint_indexes::R_ANKLE_ROLL},
    {joint_indexes::R_SHOULDER_PITCH, legacy_joint_indexes::R_SHOULDER_PITCH},
    {joint_indexes::R_SHOULDER_ROLL, legacy_joint_indexes::R_SHOULDER_ROLL},
    {joint_indexes::R_ELBOW_YAW, legacy_joint_indexes::R_ELBOW_YAW},
    {joint_indexes::R_ELBOW_ROLL, legacy_joint_indexes::R_ELBOW_ROLL},
    {joint_indexes::R_WRIST_YAW, legacy_joint_indexes::R_WRIST_YAW},
    {joint_indexes::L_HAND, legacy_joint_indexes::L_HAND},
    {joint_indexes::R_HAND, legacy_joint_indexes::R_HAND}};
} // namespace nomadz_definitions
