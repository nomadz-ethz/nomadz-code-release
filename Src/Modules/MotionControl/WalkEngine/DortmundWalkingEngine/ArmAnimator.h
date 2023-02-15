/**
 * @file ArmAnimator.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/Sensing/ArmContact.h"
#include <algorithm>

MODULE(ArmAnimator)
REQUIRES(FrameInfo)
REQUIRES(SensorData)
REQUIRES(JointData)
REQUIRES(WalkingEngineParams)
REQUIRES(MotionRequest)
REQUIRES(MotionSelection)
REQUIRES(KinematicRequest)
REQUIRES(WalkingInfo)
REQUIRES(BodyTilt)
REQUIRES(ArmContact)
PROVIDES_WITH_MODIFY(ArmMovement)
LOADS_PARAMETER(Angle, wristOffset)
LOADS_PARAMETER(Angle, handOffset)
LOADS_PARAMETER(int, timeToHoldArmBack)
LOADS_PARAMETER(Angle, armBackPitch)
LOADS_PARAMETER(Angle, armBackElbowRoll)
LOADS_PARAMETER(Angle, armBackShoulderRoll)
LOADS_PARAMETER(Angle, armToFrontShoulderRoll)
LOADS_PARAMETER(int, timeToPullPitch)
LOADS_PARAMETER(int, timeToPullArmIn)
LOADS_PARAMETER(int, timeUntilArmContactActive)
END_MODULE

class ArmAnimator : public ArmAnimatorBase {
public:
  ArmAnimator(void);
  ~ArmAnimator(void);

private:
  void update(ArmMovement& armMovement);

  unsigned leftTimeWhenNoContact, rightTimeWhenNoContact;
  unsigned leftTimeWhenContact, rightTimeWhenContact;
  ArmContact::ArmContactState lastContactStateLeft, lastContactStateRight;
  float leftPitchWhenContactStateChanged;
  float rightPitchWhenContactStateChanged;

  MotionRequest::Motion lastMotion;
  unsigned timeWhenChangedToWalk;
};
