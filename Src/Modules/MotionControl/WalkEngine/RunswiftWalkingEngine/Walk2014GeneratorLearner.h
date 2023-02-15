/**
 * @file Walk2014GeneratorLearner.h
 *
 * This file declares a module that learns joint offsets for the walking.
 *
 * This file is subject to the terms of the BHuman 2017 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Philip Reichenberg
 */

#pragma once

#include "Core/System/BHAssert.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/MotionControl/Walk2014Modifier.h"
#include "Core/Module/Module.h"
#include "Representations/MotionControl/WalkLearner.h"
#include <vector>
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Math/Random.h"

MODULE(Walk2014GeneratorLearner)

REQUIRES(InertiaSensorData)
REQUIRES(Walk2014Modifier)
PROVIDES_WITH_MODIFY(WalkLearner)
END_MODULE

class Walk2014GeneratorLearner : public Walk2014GeneratorLearnerBase {
private:
  float gyroForwardBalanceFactor;  // The current balancing value for balancing forward
  float gyroBackwardBalanceFactor; // The current balancing value for balancing backward
  float oldForwardGyro;            // The old balancing value for balancing forward
  float oldAverageMaxGyro;         // average positive max gyro.y value over the last x frames
  float oldAverageMaxGyroPhase1;   // average positive max gyro.y value over the last x frames of Phase 2
  float oldBackwardGyro;           // The old balancing value for balancing backward
  float oldAverageMinGyro;         // average negativ max gyro.y value over the last x frames
  float oldAverageMinGyroPhase1;   // average negative max gyro.y value over the last x frames of Phase 2
  float speed;                     // speed of the current walk in x-coordinate

  int phaseLearn;      // learn phase
  int learnIteration;  // learnIteration. If > theWalk2014Modifier.numOfGyroPeaks, then change balance factor
  bool isForwardPhase; // are we searching for the current positive gyro max value?
  bool wasPositiv;     // gyro changed sign

  std::vector<float> gyroForwardMax, // save the last theWalk2014Modifier.numOfGyroPeaks gyro peaks
    gyroBackwardMin;

  void update(WalkLearner& walkLearner) override;

  // learn method
  void learnGyroBalanceFactor(WalkLearner& walkLearner);

  // This method is needed so the Walk2014GeneratorLearner knows the starting balancing values and the current walking speed.
  void setBaseWalkParams(float gyroForward, float gyroBackward, float speedTransX);

public:
  Walk2014GeneratorLearner();
};
