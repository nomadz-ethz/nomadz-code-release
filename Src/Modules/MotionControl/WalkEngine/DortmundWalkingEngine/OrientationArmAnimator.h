/**
 * @file OrientationArmAnimator.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Sensing/InertiaSensorData.h"

MODULE(OrientationArmAnimator)
REQUIRES(InertiaSensorData)
REQUIRES(WalkingEngineParams)
PROVIDES_WITH_MODIFY(ArmMovement)
END_MODULE

class OrientationArmAnimator : public OrientationArmAnimatorBase {
public:
  OrientationArmAnimator(void);
  ~OrientationArmAnimator(void);

private:
  void update(ArmMovement& armMovement);
};
