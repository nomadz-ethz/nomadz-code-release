/**
 * @file OscillationController.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "OscillationController.h"
#include "Core/Math/Pose3D.h"
#include "Core/Debugging/Debugging.h"

void OscillationController::reset() {
  timeUntilSwitch = 0.f;
  angleVelAtSwitch = Vector2<>();
}

void OscillationController::predictOscillationUntilSwitch(WalkGeneratorData& generator) {
  Pose3D supportFoot = (generator.isLeftPhase) ? theRobotModel.soleRight : theRobotModel.soleLeft;
  Vector2<> pendulumSideView =
    Vector2<>(theRobotDimensions.footOuter + std::abs(supportFoot.translation.y),
              std::abs(supportFoot.translation.z) + theRobotDimensions.heightLeg5Joint + theRobotModel.centerOfMass.z);
  const float endAnglePos = std::atan2(pendulumSideView.x, pendulumSideView.y);

  anglePos[0] = Vector2<>(theOrientationData.rotation.getXAngle() + (generator.isLeftPhase ? -endAnglePos : endAnglePos),
                          theOrientationData.rotation.getYAngle());
  angleVel[0] = angleVel[0] * lowPassFilterGain +
                Vector2<>(theInertiaSensorData.gyro.x, theInertiaSensorData.gyro.y) * (1 - lowPassFilterGain);
  const float currentPendulumLength = pendulumSideView.abs() * pendulumLengthInertialFactor;
  reset();
  float timeStep = 0.005f;
  timeUntilSwitch = timeStep * maxPredictionStep;
  for (int i = 1; i < maxPredictionStep; ++i) {
    Vector2<> angleAcc =
      Vector2<>(std::sin(anglePos[i - 1].x), std::sin(anglePos[i - 1].y)) * Constants::g / currentPendulumLength;
    anglePos[i] = anglePos[i - 1] + angleVel[i - 1] * timeStep + angleAcc * 0.5f * std::pow(timeStep, 2);

    angleVel[i] = angleVel[i - 1] + angleAcc * timeStep;
    if (std::abs(anglePos[i].x) > endAnglePos) {
      timeUntilSwitch = i * timeStep;
      angleVelAtSwitch = angleVel[i - 1];
      break;
    }
  }
}
