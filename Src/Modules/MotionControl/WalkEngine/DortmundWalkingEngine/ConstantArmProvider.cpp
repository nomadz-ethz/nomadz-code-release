/**
 * @file ConstantArmProvider.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "ConstantArmProvider.h"

ConstantArmProvider::ConstantArmProvider() {
  intfac = 0;
}

ConstantArmProvider::~ConstantArmProvider() {}

void ConstantArmProvider::update(ArmMovement& armMovement) {
  armMovement.jointAngles.angles[JointData::LShoulderRoll] = -2.09f;
  armMovement.jointAngles.angles[JointData::LShoulderPitch] = 0;
  armMovement.jointAngles.angles[JointData::LElbowRoll] = -1.7f;
  armMovement.jointAngles.angles[JointData::LElbowYaw] = -1.55f;

  armMovement.jointAngles.angles[JointData::RShoulderRoll] = -2.09f;
  armMovement.jointAngles.angles[JointData::RShoulderPitch] = 0;
  armMovement.jointAngles.angles[JointData::RElbowRoll] = -1.7f;
  armMovement.jointAngles.angles[JointData::RElbowYaw] = -1.55f;

  armMovement.usearms = true;
}

MAKE_MODULE(ConstantArmProvider, dortmundWalkingEngine)
