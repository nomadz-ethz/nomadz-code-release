/**
 * @file ConstantArmProvider.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/MotionControl/MotionSelection.h"

MODULE(ConstantArmProvider)
REQUIRES(MotionSelection)
PROVIDES_WITH_MODIFY(ArmMovement)
END_MODULE

class ConstantArmProvider : public ConstantArmProviderBase {
public:
  ConstantArmProvider(void);
  ~ConstantArmProvider(void);

private:
  void update(ArmMovement& armMovement);
  float intfac;
};
