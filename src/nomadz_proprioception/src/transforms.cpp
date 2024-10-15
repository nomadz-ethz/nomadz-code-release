#include "nomadz_proprioception/transforms.hpp"

#include "nomadz_definitions/joint_indexes.hpp"
#include "nomadz_core/math/approx.hpp"
#include "nomadz_core/geometry/angle.hpp"
#include "nomadz_kinematics/forward_kinematics.hpp"
#include "nomadz_definitions/limbs.hpp"

namespace joint_indexes = nomadz_definitions::joint_indexes;
namespace side = nomadz_definitions::side;

namespace nomadz_proprioception {

  Eigen::Affine3f torsoFromSole(side::Side leg_side, const std::array<float, nomadz_kinematics::NUM_DOF>& joint_positions) {
    if (leg_side != side::LEFT && leg_side != side::RIGHT) {
      throw std::invalid_argument("Invalid leg side");
    }
    joint_indexes::JointIndexes leg_starting_index =
      leg_side == side::LEFT ? joint_indexes::L_HIP_ROLL : joint_indexes::R_HIP_ROLL;
    const Eigen::Affine3f foot = nomadz_kinematics::torsoFromFoot(leg_side == side::LEFT,
                                                                  joint_positions[joint_indexes::L_HIP_YAW_PITCH],
                                                                  joint_positions[leg_starting_index],
                                                                  joint_positions[leg_starting_index + 1],
                                                                  joint_positions[leg_starting_index + 2],
                                                                  joint_positions[leg_starting_index + 3],
                                                                  joint_positions[leg_starting_index + 4]);

    return foot * Eigen::Translation3f(0.0, 0.0, -nomadz_kinematics::FOOT_HEIGHT);
  }

  Eigen::Affine3f groundFromTorso(float torso_roll,
                                  float torso_pitch,
                                  const std::array<float, nomadz_kinematics::NUM_DOF>& joint_positions) {

    const Eigen::Affine3f transform_torso_left_sole = torsoFromSole(side::LEFT, joint_positions);
    const Eigen::Affine3f transform_torso_right_sole = torsoFromSole(side::RIGHT, joint_positions);

    // torso to ground
    const Eigen::Vector3f axis = Eigen::Vector3f(torso_roll, torso_pitch, 0.0);
    Eigen::AngleAxisf torso_rotation;
    if (const float angle = axis.norm(); nomadz_core::approx::isZero(angle)) {
      torso_rotation = Eigen::AngleAxisf::Identity();
    } else {
      torso_rotation = Eigen::AngleAxisf(angle, axis.normalized());
    }

    const Eigen::Matrix3f torso_rotation_matrix = torso_rotation.toRotationMatrix();
    const Eigen::Vector3f center_of_hip_from_left = -torso_rotation_matrix * transform_torso_left_sole.translation();
    const Eigen::Vector3f center_of_hip_from_right = -torso_rotation_matrix * transform_torso_right_sole.translation();

    const Eigen::Vector2f center_of_hip = 0.5F * (center_of_hip_from_left.head<2>() + center_of_hip_from_right.head<2>());
    const float torso_height = std::max(center_of_hip_from_left.z(), center_of_hip_from_right.z());

    Eigen::Vector3f torso_translation;
    torso_translation << center_of_hip, torso_height;

    return Eigen::Translation3f(torso_translation) * torso_rotation;
  }

  Eigen::Affine3f headFromCamera(Camera camera) {
    if (camera != Camera::UPPER_CAMERA && camera != Camera::LOWER_CAMERA) {
      throw std::invalid_argument("Invalid camera");
    }

    Eigen::Vector3f camera_offset = Eigen::Vector3f::Zero();
    float pitch = 0.F;
    if (camera == Camera::UPPER_CAMERA) {
      camera_offset =
        Eigen::Vector3f(nomadz_kinematics::UPPER_CAMERA_X_OFFSET, 0.0, nomadz_kinematics::UPPER_CAMERA_Z_OFFSET);
      pitch = nomadz_core::angle::fromDegrees(nomadz_kinematics::UPPER_CAMERA_PITCH);
    } else if (camera == Camera::LOWER_CAMERA) {
      camera_offset =
        Eigen::Vector3f(nomadz_kinematics::LOWER_CAMERA_X_OFFSET, 0.0, nomadz_kinematics::LOWER_CAMERA_Z_OFFSET);
      pitch = nomadz_core::angle::fromDegrees(nomadz_kinematics::LOWER_CAMERA_PITCH);
    }

    return Eigen::Translation3f(camera_offset) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
  }

} // namespace nomadz_proprioception
