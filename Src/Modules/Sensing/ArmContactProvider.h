/**
 * @file ArmContactProvider.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:dino.menges@tu-dortmund.de> Dino Menges</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Sensing/ArmContact.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Core/RingBufferWithSumNew.h"

MODULE(ArmContactProvider)
USES(ArmMovement)
USES(JointRequest)
REQUIRES(FallDownState)
REQUIRES(FieldDimensions)
REQUIRES(FrameInfo)
REQUIRES(GameInfo)
REQUIRES(InertiaSensorData)
REQUIRES(JointData)
REQUIRES(MotionSelection)
REQUIRES(RobotPose)
PROVIDES_WITH_MODIFY(ArmContact)
LOADS_PARAMETER(bool, enableAvoidArmContact)
LOADS_PARAMETER(bool, useRobotMap)
LOADS_PARAMETER(bool, useArmPitchDiff)
LOADS_PARAMETER(float, maxRobotDist)
LOADS_PARAMETER(Angle, maxSum)
LOADS_PARAMETER(Angle, minAngleDiff)
END_MODULE

class ArmContactProvider : public ArmContactProviderBase {
public:
  ArmContactProvider(void);
  ~ArmContactProvider(void);

private:
  void update(ArmContact& armContact);

  void resetBuffers();

  ArmContact localArmContact;
  RingBufferWithSumNew<Angle, 200> bufferLeft;
  RingBufferWithSumNew<Angle, 200> bufferRight;
  Angle lastPitchLeft, lastPitchRight;
  bool falling; // FIXME: Value is used uninitialized

  JointRequest lastRequest;
  unsigned timeWhenWalkStarted;
  MotionRequest::Motion lastMotionType;
};
