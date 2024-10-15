#pragma once

#include <array>

#include <Eigen/Geometry>

#include "nomadz_definitions/joint_indexes.hpp"
#include "nomadz_definitions/limbs.hpp"

namespace nomadz_kinematics {

  using JointAngles = std::array<float, nomadz_definitions::joint_indexes::NUM_JOINTS>;
  using LimbTransforms = std::array<Eigen::Affine3f, nomadz_definitions::limbs::NUM_LIMBS>;

} // namespace nomadz_kinematics
