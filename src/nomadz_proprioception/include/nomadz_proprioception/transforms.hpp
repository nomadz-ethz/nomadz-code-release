#pragma once

#include <array>

#include <Eigen/Geometry>

#include "nomadz_definitions/limbs.hpp"
#include "nomadz_kinematics/robot_dimensions.hpp"
#include "nomadz_proprioception/proprioception_enums.hpp"

namespace nomadz_proprioception {
  Eigen::Affine3f torsoFromSole(nomadz_definitions::side::Side leg_side,
                                const std::array<float, nomadz_kinematics::NUM_DOF>& joint_positions);

  Eigen::Affine3f
  groundFromTorso(float torso_roll, float torso_pitch, const std::array<float, nomadz_kinematics::NUM_DOF>& joint_positions);

  Eigen::Affine3f headFromCamera(Camera camera);

} // namespace nomadz_proprioception
