/**
 * @file OscillationController.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/WalkGeneratorData.h"

class OscillationController {
public:
  OscillationController(const OrientationData& theOrientationData,
                        const InertiaSensorData& theInertiaSensorData,
                        const RobotModel& theRobotModel,
                        const RobotDimensions& theRobotDimensions)
      : theOrientationData(theOrientationData), theInertiaSensorData(theInertiaSensorData), theRobotModel(theRobotModel),
        theRobotDimensions(theRobotDimensions){};
  void reset();
  void predictOscillationUntilSwitch(WalkGeneratorData& generator);

private:
  // This magical number is used to incorporate the inertial effect of the robot in the inverted pendulum, the factor
  // should corresponding to mr/I .
  static constexpr int maxPredictionStep = 50;
  static constexpr float pendulumLengthInertialFactor = 1.3f;
  const float lowPassFilterGain = 0.6f;

  const OrientationData& theOrientationData;
  const InertiaSensorData& theInertiaSensorData;
  const RobotModel& theRobotModel;
  const RobotDimensions& theRobotDimensions;

  Vector2<> anglePos[maxPredictionStep];
  Vector2<> angleVel[maxPredictionStep];
  float timeUntilSwitch;
  Vector2<> angleVelAtSwitch;
};
