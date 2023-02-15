/**
 * @file OrientationArmAnimator.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "OrientationArmAnimator.h"

OrientationArmAnimator::OrientationArmAnimator() {}

OrientationArmAnimator::~OrientationArmAnimator() {}

void OrientationArmAnimator::update(ArmMovement& armMovement) {
  armMovement.jointAngles.angles[JointData::LShoulderPitch] = 0;
  armMovement.jointAngles.angles[JointData::LShoulderRoll] = (float)(theWalkingEngineParams.arms1 * 3.1415 / 180);
  armMovement.jointAngles.angles[JointData::LElbowYaw] = 0;
  armMovement.jointAngles.angles[JointData::LElbowRoll] = 0;

  armMovement.jointAngles.angles[JointData::RShoulderPitch] = 0;
  armMovement.jointAngles.angles[JointData::RShoulderRoll] = (float)(theWalkingEngineParams.arms1 * 3.1415 / 180);
  armMovement.jointAngles.angles[JointData::LElbowYaw] = 0;
  armMovement.jointAngles.angles[JointData::LElbowRoll] = 0;

  float leftArm, rightArm;

  rightArm = leftArm = -90 * 3.1415f / 180 - theWalkingEngineParams.armFactor * ((float)theInertiaSensorData.angle.y);
  armMovement.jointAngles.angles[JointData::LShoulderPitch] = leftArm;
  armMovement.jointAngles.angles[JointData::RShoulderPitch] = rightArm;
  armMovement.usearms = true;
}

MAKE_MODULE(OrientationArmAnimator, dortmundWalkingEngine)
