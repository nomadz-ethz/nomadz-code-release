#include "nomadz_kinematics/robot_model.hpp"

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nomadz_definitions/limbs.hpp"
#include "nomadz_definitions/joint_indexes.hpp"

#include "nomadz_kinematics/forward_kinematics.hpp"

namespace fs = std::filesystem;
namespace limbs = nomadz_definitions::limbs;
using LimbsArray = std::array<Eigen::Affine3f, nomadz_definitions::limbs::NUM_LIMBS>;

namespace nomadz_kinematics {
  RobotModel::RobotModel() {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nomadz_kinematics");
    std::string config_file_path = fs::path(package_share_directory) / fs::path("config/mass_calibration.yaml");
    body_parts_ = parseMassCalibration(config_file_path);

    limbs_[limbs::TORSO] = Eigen::Affine3f::Identity();
  }

  LimbsArray RobotModel::getLimbs() const {
    return limbs_;
  }

  float RobotModel::getTotalMass() {
    return TOTAL_MASS;
  }

  Eigen::Vector3f RobotModel::getCenterOfMass() const {
    return center_of_mass_;
  }

  Eigen::Matrix3f RobotModel::getInertiaMatrix() const {
    return inertia_matrix_;
  }

  void RobotModel::setJointPositions(const JointAngles& joint_positions) {
    calculateTorsoFromArmLimbs(true, joint_positions, limbs_);
    calculateTorsoFromArmLimbs(false, joint_positions, limbs_);
    calculateTorsoFromLegLimbs(true, joint_positions, limbs_);
    calculateTorsoFromLegLimbs(false, joint_positions, limbs_);
    calculateTorsoFromHeadLimbs(joint_positions, limbs_);

    updateCenterOfMass();
    updateInertiaMatrix();
  }

  void RobotModel::updateCenterOfMass() {
    center_of_mass_.setZero();
    for (int i = 0; i < limbs::NUM_LIMBS; i++) {
      center_of_mass_ += body_parts_[i].mass * (limbs_[i] * body_parts_[i].offset);
    }
    center_of_mass_ /= TOTAL_MASS;
  }

  void RobotModel::updateInertiaMatrix() {
    // TODO(Zichong): Implement this function
    inertia_matrix_.setZero();
    inertia_matrix_ += fillMatrixFromLT(body_parts_[limbs::TORSO].inertia_matrix_lt);
  }

  Eigen::Matrix3f RobotModel::fillMatrixFromLT(const Eigen::Matrix<float, 6, 1>& lt) {
    Eigen::Matrix3f matrix;
    matrix << lt(0), lt(3), lt(5), lt(3), lt(1), lt(4), lt(5), lt(4), lt(2);
    return matrix;
  }

  Eigen::Affine3f RobotModel::getFootPose(bool left) const {
    return getLimbs()[left ? nomadz_definitions::limbs::FOOT_LEFT : nomadz_definitions::limbs::FOOT_RIGHT];
  }

} // namespace nomadz_kinematics
