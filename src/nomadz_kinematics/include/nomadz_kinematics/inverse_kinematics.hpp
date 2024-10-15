#pragma once

#include <functional>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "nomadz_kinematics/robot_dimensions.hpp"

namespace nomadz_kinematics {

  float normalizeAngle(float angle);

  void inverseTorsoFromHead(const Eigen::Affine3f& target_pose, float& head_yaw, float& head_pitch);

  void inverseTorsoFromFoot(bool left,
                            const Eigen::Affine3f& target_pose,
                            float& hip_yaw_pitch,
                            float& hip_roll,
                            float& hip_pitch,
                            float& knee_pitch,
                            float& ankle_pitch,
                            float& ankle_roll);
  void inverseTorsoFromFoot(const Eigen::Affine3f& leftTarget,
                            const Eigen::Affine3f& rightTarget,
                            std::array<float, NUM_DOF>& current_joint_position);
}; // namespace nomadz_kinematics
