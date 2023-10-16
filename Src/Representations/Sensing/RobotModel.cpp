/**
 * @file RobotModel.cpp
 *
 * Implementation of class RobotModel.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Alexander Härtl
 */

#include "RobotModel.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "Tools/Motion/ForwardKinematic.h"

RobotModel::RobotModel(const JointData& joints,
                       const RobotDimensions& robotDimensions,
                       const MassCalibration& massCalibration)
    : RobotModel() {
  setJointData(joints, robotDimensions, massCalibration);
}

void RobotModel::setJointData(const JointData& joints,
                              const RobotDimensions& robotDimensions,
                              const MassCalibration& massCalibration) {
  ForwardKinematic::calculateHeadChain(joints, robotDimensions, limbs);

  for (int side = 0; side < 2; side++) {
    const bool left = side == 0;
    ForwardKinematic::calculateArmChain(left, joints, robotDimensions, limbs);
    ForwardKinematic::calculateLegChain(left, joints, robotDimensions, limbs);
  }

  soleLeft = limbs[Limbs::footLeft] + Vector3<>(0.f, 0.f, -robotDimensions.heightLeg5Joint);
  soleRight = limbs[Limbs::footRight] + Vector3<>(0.f, 0.f, -robotDimensions.heightLeg5Joint);

  // calculate center of mass
  updateCenterOfMass(massCalibration);
}

void RobotModel::updateCenterOfMass(const MassCalibration& massCalibration) {
  // calculate center of mass
  centerOfMass = Vector3<>(0, 0, 0);
  for (int i = 0; i < Limbs::numOfLimbs; i++) {
    const MassCalibration::MassInfo& limb = massCalibration.masses[i];
    centerOfMass += (limbs[i] * limb.offset) * limb.mass;
  }
  centerOfMass /= massCalibration.totalMass;
}

void RobotModel::draw() const {
  DECLARE_DEBUG_DRAWING3D("representation:RobotModel", "origin");
  COMPLEX_DRAWING3D("representation:RobotModel", {
    for (int i = 0; i < MassCalibration::numOfLimbs; ++i) {
      const Pose3D& p = limbs[i];
      const int vectorLen = 30;
      int vectorWidth = 1;
      const Vector3<>& v = p.translation;
      const Vector3<> v1 = p * Vector3<>(vectorLen, 0, 0);
      const Vector3<> v2 = p * Vector3<>(0, vectorLen, 0);
      const Vector3<> v3 = p * Vector3<>(0, 0, vectorLen);
      if (i == MassCalibration::numOfLimbs - 1) {
        vectorWidth = 2;
      }
      SPHERE3D("representation:RobotModel", v.x, v.y, v.z, vectorWidth * 2, ColorRGBA(255, 0, 0));
      LINE3D("representation:RobotModel", v.x, v.y, v.z, v1.x, v1.y, v1.z, vectorWidth, ColorRGBA(255, 0, 0));
      LINE3D("representation:RobotModel", v.x, v.y, v.z, v2.x, v2.y, v2.z, vectorWidth, ColorRGBA(0, 255, 0));
      LINE3D("representation:RobotModel", v.x, v.y, v.z, v3.x, v3.y, v3.z, vectorWidth, ColorRGBA(0, 0, 255));
    }
  });
  DECLARE_DEBUG_DRAWING3D("representation:RobotModel:centerOfMass", "origin");
  int vectorWidth = 2;
  SPHERE3D("representation:RobotModel:centerOfMass",
           centerOfMass.x,
           centerOfMass.y,
           centerOfMass.z,
           vectorWidth * 3,
           ColorRGBA(0, 0, 255));
  for (int i = 0; i < MassCalibration::numOfLimbs; ++i) {
    if (i == MassCalibration::footLeft || i == MassCalibration::footRight || i == MassCalibration::torso) {
      const Pose3D& p = limbs[i];
      const int vectorLen = 30;
      const Vector3<>& v = p.translation;
      const Vector3<> v1 = p * Vector3<>(vectorLen, 0, 0);
      const Vector3<> v2 = p * Vector3<>(0, vectorLen, 0);
      const Vector3<> v3 = p * Vector3<>(0, 0, vectorLen);
      SPHERE3D("representation:RobotModel:centerOfMass", v.x, v.y, v.z, vectorWidth * 2, ColorRGBA(255, 0, 0));

      LINE3D("representation:RobotModel:centerOfMass", v.x, v.y, v.z, v1.x, v1.y, v1.z, vectorWidth, ColorRGBA(255, 0, 0));
      LINE3D("representation:RobotModel:centerOfMass", v.x, v.y, v.z, v2.x, v2.y, v2.z, vectorWidth, ColorRGBA(0, 255, 0));
      LINE3D("representation:RobotModel:centerOfMass", v.x, v.y, v.z, v3.x, v3.y, v3.z, vectorWidth, ColorRGBA(0, 0, 255));
    }
  }
}
