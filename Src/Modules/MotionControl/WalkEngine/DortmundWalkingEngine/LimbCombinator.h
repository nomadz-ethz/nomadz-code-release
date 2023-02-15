/**
 * @file LimbCombinator.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Core/RingBuffer.h"
#include "Tools/DortmundWalkingEngine/Filters/FastFilter.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/JointData.h"

MODULE(LimbCombinator)
REQUIRES(SpeedInfo)
REQUIRES(KinematicOutput)
REQUIRES(ArmMovement)
REQUIRES(SensorData)
REQUIRES(JointData)
REQUIRES(WalkingInfo)
REQUIRES(MotionRequest)
REQUIRES(WalkingEngineParams)
PROVIDES_WITH_MODIFY(WalkingEngineOutput)
REQUIRES(WalkingEngineOutput)
PROVIDES_WITH_MODIFY(WalkingEngineStandOutput)
END_MODULE

class LimbCombinator : public LimbCombinatorBase {
public:
  LimbCombinator(void);
  ~LimbCombinator(void);

  static int walkingEngineTime;

private:
  float getOffset(int j, float targetAngle);
  float angleoffset[JointData::numOfJoints][200];
  RingBuffer<float, 100> delayBuffer[JointData::numOfJoints];
  void update(WalkingEngineOutput& walkingEngineOutput);
  void update(WalkingEngineStandOutput& walkingEngineStandOutput) {
    (JointRequest&)walkingEngineStandOutput = theWalkingEngineOutput.jointRequest;
  }
  bool init;
  FastFilter<float> filter[JointData::numOfJoints];
};
