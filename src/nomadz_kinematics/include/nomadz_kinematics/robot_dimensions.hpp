#pragma once

namespace nomadz_kinematics {
  constexpr int NUM_DOF = 25;
  constexpr float NECK_OFFSET_Z = 0.1265F;

  constexpr float SHOULDER_OFFSET_Y = 0.098F;
  constexpr float SHOULDER_OFFSET_Z = 0.1F;
  constexpr float UPPER_ARM_LENGTH = 0.105F;
  constexpr float ELBOW_OFFSET_Y = 0.015F;
  constexpr float LOWER_ARM_LENGTH = 0.05595F;
  constexpr float HAND_OFFSET_X = 0.05775F;
  constexpr float HAND_OFFSET_Z = 0.01231F;

  constexpr float HIP_OFFSET_Y = 0.05F;
  constexpr float HIP_OFFSET_Z = 0.085F;
  constexpr float THIGH_LENGTH = 0.1F;
  constexpr float TIBIA_LENGTH = 0.1029F;
  constexpr float FOOT_HEIGHT = 0.04519F;

  constexpr float UPPER_CAMERA_X_OFFSET = 0.05871F;
  constexpr float UPPER_CAMERA_Z_OFFSET = 0.06364F;
  constexpr float UPPER_CAMERA_PITCH = 1.2F;

  constexpr float LOWER_CAMERA_X_OFFSET = 0.05071F;
  constexpr float LOWER_CAMERA_Z_OFFSET = 0.01774F;
  constexpr float LOWER_CAMERA_PITCH = 39.7F;
}; // namespace nomadz_kinematics
