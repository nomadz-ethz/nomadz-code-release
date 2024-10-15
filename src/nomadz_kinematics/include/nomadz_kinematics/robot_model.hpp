#pragma once

#include <array>
#include <vector>
#include <Eigen/Core>

#include "nomadz_kinematics/mass_calibration_parser.hpp"
#include "nomadz_kinematics/types.hpp"

namespace nomadz_kinematics {
  class RobotModel {
  public:
    RobotModel();

    LimbTransforms getLimbs() const;
    static float getTotalMass();
    Eigen::Vector3f getCenterOfMass() const;
    Eigen::Matrix3f getInertiaMatrix() const;
    void setJointPositions(const JointAngles& joint_positions);
    Eigen::Affine3f getFootPose(bool left) const;

  private:
    static constexpr float TOTAL_MASS = 5.30539F;

    std::vector<BodyPart> body_parts_;

    LimbTransforms limbs_;

    Eigen::Vector3f center_of_mass_;
    Eigen::Matrix3f inertia_matrix_;

    void updateCenterOfMass();
    void updateInertiaMatrix();
    static Eigen::Matrix3f fillMatrixFromLT(const Eigen::Matrix<float, 6, 1>& lt);
  };
} // namespace nomadz_kinematics
