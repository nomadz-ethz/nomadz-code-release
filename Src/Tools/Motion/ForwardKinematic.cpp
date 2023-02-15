/**
 * @file ForwardKinematic.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A> and
 * <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#include "ForwardKinematic.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Core/Math/BHMath.h"
#include "Core/Math/Pose3D.h"
#include "Core/Math/RotationMatrix.h"

void ForwardKinematic::calculateArmChain(bool left,
                                         const JointData& joints,
                                         const RobotDimensions& robotDimensions,
                                         Pose3D limbs[Limbs::numOfLimbs]) {
  int sign = left ? 1 : -1;
  Limbs::Limb shoulderLimb = left ? Limbs::shoulderLeft : Limbs::shoulderRight;
  JointData::Joint shoulderJoint = left ? JointData::LShoulderPitch : JointData::RShoulderPitch;

  Pose3D& shoulder = limbs[shoulderLimb];
  Pose3D& biceps = limbs[shoulderLimb + 1];
  Pose3D& elbow = limbs[shoulderLimb + 2];
  Pose3D& foreArm = limbs[shoulderLimb + 3];
  Pose3D& wrist = limbs[shoulderLimb + 4];

  shoulder = Pose3D(robotDimensions.armOffset.x, robotDimensions.armOffset.y * sign, robotDimensions.armOffset.z) *=
    RotationMatrix::fromRotationY(joints.angles[shoulderJoint]);
  biceps = shoulder * RotationMatrix::fromRotationZ(joints.angles[shoulderJoint + 1]);
  elbow = (biceps + Vector3<>(robotDimensions.upperArmLength, robotDimensions.yElbowShoulder * sign, 0)) *=
    RotationMatrix::fromRotationX(joints.angles[shoulderJoint + 2]);
  foreArm = elbow * RotationMatrix::fromRotationZ(joints.angles[shoulderJoint + 3]);
  wrist = (foreArm + Vector3<>(robotDimensions.xOffsetElbowToWrist, 0, 0)) *=
    RotationMatrix::fromRotationX(joints.angles[shoulderJoint + 4]);
}

void ForwardKinematic::calculateLegChain(bool left,
                                         const JointData& joints,
                                         const RobotDimensions& robotDimensions,
                                         Pose3D limbs[Limbs::numOfLimbs]) {
  int sign = left ? 1 : -1;
  Limbs::Limb pelvisLimb = left ? Limbs::pelvisLeft : Limbs::pelvisRight;
  JointData::Joint hipJoint = left ? JointData::LHipYawPitch : JointData::RHipYawPitch;

  Pose3D& pelvis = limbs[pelvisLimb];
  Pose3D& hip = limbs[pelvisLimb + 1];
  Pose3D& thigh = limbs[pelvisLimb + 2];
  Pose3D& tibia = limbs[pelvisLimb + 3];
  Pose3D& ankle = limbs[pelvisLimb + 4];
  Pose3D& foot = limbs[pelvisLimb + 5];

  pelvis = Pose3D(0.f, robotDimensions.yHipOffset * sign, 0.f) *
           (RotationMatrix::fromRotationX(pi_4 * sign) * RotationMatrix::fromRotationZ(joints.angles[hipJoint] * -sign) *
            RotationMatrix::fromRotationX(-pi_4 * sign));
  hip = pelvis * RotationMatrix::fromRotationX(joints.angles[hipJoint + 1]);
  thigh = hip * RotationMatrix::fromRotationY(joints.angles[hipJoint + 2]);
  tibia = (thigh + Vector3<>(0, 0, -robotDimensions.upperLegLength)) *=
    RotationMatrix::fromRotationY(joints.angles[hipJoint + 3]);
  ankle = (tibia + Vector3<>(0, 0, -robotDimensions.lowerLegLength)) *=
    RotationMatrix::fromRotationY(joints.angles[hipJoint + 4]);
  foot = ankle * RotationMatrix::fromRotationX(joints.angles[hipJoint + 5]);
}

void ForwardKinematic::calculateHeadChain(const JointData& joints,
                                          const RobotDimensions& robotDimensions,
                                          Pose3D limbs[Limbs::numOfLimbs]) {
  limbs[Limbs::neck] = Pose3D(0.f, 0.f, robotDimensions.zLegJoint1ToHeadPan) *=
    RotationMatrix::fromRotationZ(joints.angles[JointData::HeadYaw]);
  limbs[Limbs::head] = limbs[Limbs::neck] * RotationMatrix::fromRotationY(joints.angles[JointData::HeadPitch]);
}
