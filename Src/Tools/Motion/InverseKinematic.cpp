/**
 * @file InverseKinematic.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Alexander Härtl and Jesse Richter-Klug
 */

#include "InverseKinematic.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointData.h"
#include "Core/RangeNew.h"
#include "Core/Math/BHMath.h"
#include "Core/Math/Pose3D.h"

bool InverseKinematic::calcLegJoints(const Pose3D& positionLeft,
                                     const Pose3D& positionRight,
                                     JointData& jointAngles,
                                     const RobotDimensions& robotDimensions,
                                     float ratio) {
  static const Pose3D rotPi_4 = RotationMatrix::fromRotationX(pi_4);
  static const Pose3D rotMinusPi_4 = RotationMatrix::fromRotationX(-pi_4);
  const RangeNewf& cosClipping = RangeNewf::OneRangeNew();

  ratio = RangeNewf::ZeroOneRangeNew().limit(ratio);

  const Pose3D lTarget0 = (rotMinusPi_4 + Vector3<>(0.f, -robotDimensions.yHipOffset, 0.f)) *= positionLeft;
  const Pose3D rTarget0 = (rotPi_4 + Vector3<>(0.f, robotDimensions.yHipOffset, 0.f)) *= positionRight;
  const Vector3<> lFootToHip = lTarget0.rotation.invert() * -lTarget0.translation;
  const Vector3<> rFootToHip = rTarget0.rotation.invert() * -rTarget0.translation;
  const float lMinusJoint5 = std::atan2(lFootToHip.y, lFootToHip.z);
  const float rJoint5 = std::atan2(rFootToHip.y, rFootToHip.z);
  const float lMinusBetaAndJoint4 = -std::atan2(lFootToHip.x, std::sqrt(sqr(lFootToHip.y) + sqr(lFootToHip.z)));
  const float rMinusBetaAndJoint4 = -std::atan2(rFootToHip.x, std::sqrt(sqr(rFootToHip.y) + sqr(rFootToHip.z)));
  const Vector3<> lHipRotationC1 = lTarget0.rotation * (RotationMatrix::fromRotationX(-lMinusJoint5) *
                                                        RotationMatrix::fromRotationY(-lMinusBetaAndJoint4))[1];
  const Vector3<> rHipRotationC1 =
    rTarget0.rotation * (RotationMatrix::fromRotationX(-rJoint5) * RotationMatrix::fromRotationY(-rMinusBetaAndJoint4))[1];
  const float lMinusJoint0 = std::atan2(-lHipRotationC1.x, lHipRotationC1.y);
  const float rJoint0 = std::atan2(-rHipRotationC1.x, rHipRotationC1.y);
  const float lJoint0Combined = -lMinusJoint0 * ratio + rJoint0 * (1.f - ratio);

  const Pose3D lTarget1 = Pose3D(RotationMatrix::fromRotationZ(lJoint0Combined)) *= lTarget0;
  const Pose3D rTarget1 = Pose3D(RotationMatrix::fromRotationZ(-lJoint0Combined)) *= rTarget0;
  const Vector3<>& lHipToFoot = lTarget1.translation;
  const Vector3<>& rHipToFoot = rTarget1.translation;
  const float lMinusPi_4MinusJoint1 = -std::atan2(-lHipToFoot.y, -lHipToFoot.z);
  const float rPi_4AndJoint1 = -std::atan2(-rHipToFoot.y, -rHipToFoot.z);
  const float lJoint2MinusAlpha =
    std::atan2(-lHipToFoot.x, std::sqrt(sqr(lHipToFoot.y) + sqr(lHipToFoot.z)) * -sgn(lHipToFoot.z));
  const float rJoint2MinusAlpha =
    std::atan2(-rHipToFoot.x, std::sqrt(sqr(rHipToFoot.y) + sqr(rHipToFoot.z)) * -sgn(rHipToFoot.z));
  const Vector3<> lFootRotationC2 = RotationMatrix::fromRotationY(-lJoint2MinusAlpha) *
                                    RotationMatrix::fromRotationX(-lMinusPi_4MinusJoint1) * lTarget1.rotation.c2;
  const Vector3<> rFootRotationC2 = RotationMatrix::fromRotationY(-rJoint2MinusAlpha) *
                                    RotationMatrix::fromRotationX(-rPi_4AndJoint1) * rTarget1.rotation.c2;
  const float h1 = robotDimensions.upperLegLength;
  const float h2 = robotDimensions.lowerLegLength;
  const float hl = lTarget1.translation.abs();
  const float hr = rTarget1.translation.abs();
  const float h1Sqr = h1 * h1;
  const float h2Sqr = h2 * h2;
  const float hlSqr = hl * hl;
  const float hrSqr = hr * hr;
  const float lCosMinusAlpha = (h1Sqr + hlSqr - h2Sqr) / (2.f * h1 * hl);
  const float rCosMinusAlpha = (h1Sqr + hrSqr - h2Sqr) / (2.f * h1 * hr);
  const float lCosMinusBeta = (h2Sqr + hlSqr - h1Sqr) / (2.f * h2 * hl);
  const float rCosMinusBeta = (h2Sqr + hrSqr - h1Sqr) / (2.f * h2 * hr);
  const float lAlpha = -std::acos(cosClipping.limit(lCosMinusAlpha));
  const float rAlpha = -std::acos(cosClipping.limit(rCosMinusAlpha));
  const float lBeta = -std::acos(cosClipping.limit(lCosMinusBeta));
  const float rBeta = -std::acos(cosClipping.limit(rCosMinusBeta));

  jointAngles.angles[JointData::LHipYawPitch] = lJoint0Combined;
  jointAngles.angles[JointData::LHipRoll] = (lMinusPi_4MinusJoint1 + pi_4);
  jointAngles.angles[JointData::LHipPitch] = lJoint2MinusAlpha + lAlpha;
  jointAngles.angles[JointData::LKneePitch] = -lAlpha - lBeta;
  jointAngles.angles[JointData::LAnklePitch] = std::atan2(lFootRotationC2.x, lFootRotationC2.z) + lBeta;
  jointAngles.angles[JointData::LAnkleRoll] = std::asin(-lFootRotationC2.y);

  jointAngles.angles[JointData::RHipYawPitch] = lJoint0Combined;
  jointAngles.angles[JointData::RHipRoll] = rPi_4AndJoint1 - pi_4;
  jointAngles.angles[JointData::RHipPitch] = rJoint2MinusAlpha + rAlpha;
  jointAngles.angles[JointData::RKneePitch] = -rAlpha - rBeta;
  jointAngles.angles[JointData::RAnklePitch] = std::atan2(rFootRotationC2.x, rFootRotationC2.z) + rBeta;
  jointAngles.angles[JointData::RAnkleRoll] = std::asin(-rFootRotationC2.y);
  const float maxLen = h1 + h2;
  return hl <= maxLen && hr <= maxLen;
}

bool InverseKinematic::calcLegJoints(const Pose3D& positionLeft,
                                     const Pose3D& positionRight,
                                     const Vector2<>& bodyRotation,
                                     JointData& jointAngles,
                                     const RobotDimensions& robotDimensions,
                                     float ratio) {
  const Eigen::Quaternionf bodyRot = Rotation::aroundX(bodyRotation.x) * Rotation::aroundY(bodyRotation.y);
  return calcLegJoints(positionLeft, positionRight, bodyRot, jointAngles, robotDimensions, ratio);
}

bool InverseKinematic::calcLegJoints(const Pose3D& positionLeft,
                                     const Pose3D& positionRight,
                                     const Eigen::Quaternionf& bodyRotation,
                                     JointData& jointAngles,
                                     const RobotDimensions& robotDimensions,
                                     float ratio) {
  static const Pose3D rotPi_4 = RotationMatrix::fromRotationX(pi_4);
  static const Pose3D rotMinusPi_4 = RotationMatrix::fromRotationX(-pi_4);
  const RangeNewf& cosClipping = RangeNewf::OneRangeNew();

  ratio = RangeNewf::ZeroOneRangeNew().limit(ratio);

  const Pose3D lTarget0 =
    (((rotMinusPi_4 + Vector3<>(0.f, -robotDimensions.yHipOffset, 0.f)) *= eigenQuatToRotmat(bodyRotation.inverse())) *=
     positionLeft) += Vector3<>(0.f, 0.f, robotDimensions.heightLeg5Joint);
  const Pose3D rTarget0 =
    (((rotPi_4 + Vector3<>(0.f, robotDimensions.yHipOffset, 0.f)) *= eigenQuatToRotmat(bodyRotation.inverse())) *=
     positionRight) += Vector3<>(0.f, 0.f, robotDimensions.heightLeg5Joint);
  const Vector3<> lFootToHip = lTarget0.rotation.invert() * -lTarget0.translation;
  const Vector3<> rFootToHip = rTarget0.rotation.invert() * -rTarget0.translation;
  const float lMinusJoint5 = std::atan2(lFootToHip.y, lFootToHip.z);
  const float rJoint5 = std::atan2(rFootToHip.y, rFootToHip.z);
  const float lMinusBetaAndJoint4 = -std::atan2(lFootToHip.x, std::sqrt(sqr(lFootToHip.y) + sqr(lFootToHip.z)));
  const float rMinusBetaAndJoint4 = -std::atan2(rFootToHip.x, std::sqrt(sqr(rFootToHip.y) + sqr(rFootToHip.z)));
  const Vector3<> lHipRotationC1 = lTarget0.rotation * (RotationMatrix::fromRotationX(-lMinusJoint5) *
                                                        RotationMatrix::fromRotationY(-lMinusBetaAndJoint4))[1];
  const Vector3<> rHipRotationC1 =
    rTarget0.rotation * (RotationMatrix::fromRotationX(-rJoint5) * RotationMatrix::fromRotationY(-rMinusBetaAndJoint4))[1];
  const float lMinusJoint0 = std::atan2(-lHipRotationC1.x, lHipRotationC1.y);
  const float rJoint0 = std::atan2(-rHipRotationC1.x, rHipRotationC1.y);
  const float lJoint0Combined = -lMinusJoint0 * ratio + rJoint0 * (1.f - ratio);

  const Pose3D lTarget1 = Pose3D(RotationMatrix::fromRotationZ(lJoint0Combined)) *= lTarget0;
  const Pose3D rTarget1 = Pose3D(RotationMatrix::fromRotationZ(-lJoint0Combined)) *= rTarget0;
  const Vector3<>& lHipToFoot = lTarget1.translation;
  const Vector3<>& rHipToFoot = rTarget1.translation;
  const float lMinusPi_4MinusJoint1 = -std::atan2(-lHipToFoot.y, -lHipToFoot.z);
  const float rPi_4AndJoint1 = -std::atan2(-rHipToFoot.y, -rHipToFoot.z);
  const float lJoint2MinusAlpha =
    std::atan2(-lHipToFoot.x, std::sqrt(sqr(lHipToFoot.y) + sqr(lHipToFoot.z)) * -sgn(lHipToFoot.z));
  const float rJoint2MinusAlpha =
    std::atan2(-rHipToFoot.x, std::sqrt(sqr(rHipToFoot.y) + sqr(rHipToFoot.z)) * -sgn(rHipToFoot.z));
  const Vector3<> lFootRotationC2 = RotationMatrix::fromRotationY(-lJoint2MinusAlpha) *
                                    RotationMatrix::fromRotationX(-lMinusPi_4MinusJoint1) * lTarget1.rotation.c2;
  const Vector3<> rFootRotationC2 = RotationMatrix::fromRotationY(-rJoint2MinusAlpha) *
                                    RotationMatrix::fromRotationX(-rPi_4AndJoint1) * rTarget1.rotation.c2;
  const float h1 = robotDimensions.upperLegLength;
  const float h2 = robotDimensions.lowerLegLength;
  const float hl = lTarget1.translation.abs();
  const float hr = rTarget1.translation.abs();
  const float h1Sqr = h1 * h1;
  const float h2Sqr = h2 * h2;
  const float hlSqr = hl * hl;
  const float hrSqr = hr * hr;
  const float lCosMinusAlpha = (h1Sqr + hlSqr - h2Sqr) / (2.f * h1 * hl);
  const float rCosMinusAlpha = (h1Sqr + hrSqr - h2Sqr) / (2.f * h1 * hr);
  const float lCosMinusBeta = (h2Sqr + hlSqr - h1Sqr) / (2.f * h2 * hl);
  const float rCosMinusBeta = (h2Sqr + hrSqr - h1Sqr) / (2.f * h2 * hr);
  const float lAlpha = -std::acos(cosClipping.limit(lCosMinusAlpha));
  const float rAlpha = -std::acos(cosClipping.limit(rCosMinusAlpha));
  const float lBeta = -std::acos(cosClipping.limit(lCosMinusBeta));
  const float rBeta = -std::acos(cosClipping.limit(rCosMinusBeta));

  jointAngles.angles[JointData::LHipYawPitch] = lJoint0Combined;
  jointAngles.angles[JointData::LHipRoll] = (lMinusPi_4MinusJoint1 + pi_4);
  jointAngles.angles[JointData::LHipPitch] = lJoint2MinusAlpha + lAlpha;
  jointAngles.angles[JointData::LKneePitch] = -lAlpha - lBeta;
  jointAngles.angles[JointData::LAnklePitch] = std::atan2(lFootRotationC2.x, lFootRotationC2.z) + lBeta;
  jointAngles.angles[JointData::LAnkleRoll] = std::asin(-lFootRotationC2.y);

  jointAngles.angles[JointData::RHipYawPitch] = lJoint0Combined;
  jointAngles.angles[JointData::RHipRoll] = rPi_4AndJoint1 - pi_4;
  jointAngles.angles[JointData::RHipPitch] = rJoint2MinusAlpha + rAlpha;
  jointAngles.angles[JointData::RKneePitch] = -rAlpha - rBeta;
  jointAngles.angles[JointData::RAnklePitch] = std::atan2(rFootRotationC2.x, rFootRotationC2.z) + rBeta;
  jointAngles.angles[JointData::RAnkleRoll] = std::asin(-rFootRotationC2.y);
  const float maxLen = h1 + h2;
  return hl <= maxLen && hr <= maxLen;
}

void InverseKinematic::calcHeadJoints(const Vector3<>& position,
                                      const float imageTilt,
                                      const RobotDimensions& robotDimensions,
                                      const bool lowerCamera,
                                      Vector2<>& panTilt,
                                      const CameraCalibration& cameraCalibration) {
  const Vector2<> headJoint2Target(std::sqrt(sqr(position.x) + sqr(position.y)),
                                   position.z - robotDimensions.zLegJoint1ToHeadPan);
  const Vector2<> headJoint2Camera(robotDimensions.getXHeadTiltToCamera(lowerCamera),
                                   robotDimensions.getZHeadTiltToCamera(lowerCamera));
  const float headJoint2CameraAngle = std::atan2(headJoint2Camera.x, headJoint2Camera.y);
  const float cameraAngle =
    pi3_2 - imageTilt - (pi_2 - headJoint2CameraAngle) - robotDimensions.getHeadTiltToCameraTilt(lowerCamera);
  const float targetAngle = std::asin(headJoint2Camera.abs() * std::sin(cameraAngle) / headJoint2Target.abs());
  const float headJointAngle = pi - targetAngle - cameraAngle;
  panTilt.y = std::atan2(headJoint2Target.x, headJoint2Target.y) - headJointAngle - headJoint2CameraAngle;
  panTilt.x = std::atan2(position.y, position.x);
  if (lowerCamera) {
    panTilt.x -= cameraCalibration.cameraPanCorrection;
    panTilt.y -= cameraCalibration.cameraTiltCorrection;
  } else {
    panTilt.x -= cameraCalibration.upperCameraPanCorrection;
    panTilt.y -= cameraCalibration.upperCameraTiltCorrection;
  }
}

RotationMatrix InverseKinematic::eigenQuatToRotmat(Eigen::Quaternionf quat) {
  RotationMatrix R;
  R.c0[0] = sqr_bh(quat.w()) + sqr_bh(quat.x()) - sqr_bh(quat.y()) - sqr_bh(quat.z());
  R.c0[1] = 2 * quat.w() * quat.z() + 2 * quat.x() * quat.y();
  R.c0[2] = 2 * quat.x() * quat.z() - 2 * quat.w() * quat.y();
  R.c1[0] = 2 * quat.x() * quat.y() - 2 * quat.w() * quat.z();
  R.c1[1] = sqr_bh(quat.w()) - sqr_bh(quat.x()) + sqr_bh(quat.y()) - sqr_bh(quat.z());
  R.c1[2] = 2 * quat.w() * quat.x() + 2 * quat.y() * quat.z();
  R.c2[0] = 2 * quat.w() * quat.y() + 2 * quat.x() * quat.z();
  R.c2[1] = 2 * quat.y() * quat.z() - 2 * quat.w() * quat.x();
  R.c2[2] = sqr_bh(quat.w()) - sqr_bh(quat.x()) - sqr_bh(quat.y()) + sqr_bh(quat.z());

  return R;
}
