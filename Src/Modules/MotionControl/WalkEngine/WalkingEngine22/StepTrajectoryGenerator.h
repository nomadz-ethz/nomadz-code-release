/**
 * @file StepTrajectoryGenerator.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once
#include "Representations/MotionControl/WalkGeneratorData.h"
#include "Core/Range.h"

float parabolicReturn(float f) {
  Rangef::ZeroOneRange().clamp(f);

  if (f < 0.25f) {
    return 8.f * f * f;
  } else if (f < 0.75f) {
    float x = f - 0.5f;
    return 1.f - 8.f * x * x;
  } else {
    float x = 1.f - f;
    return 8.f * x * x;
  }
}

float parabolicStep(float time, float period) {
  float timeFraction = Rangef::ZeroOneRange().limit(time / period);
  if (timeFraction < 0.5f) {
    return 2.f * timeFraction * timeFraction;
  } else {
    return 4.f * timeFraction - 2.f * timeFraction * timeFraction - 1.f;
  }
}

Vector3<> bezierQuadratic(float t, Vector3<> P0, Vector3<> P1, Vector3<> P2) {
  return P0 * std::pow((1 - t), 2) + P1 * 2 * (1 - t) * t + P2 * std::pow(t, 2);
}

Vector3<> bezierCubic(float t, Vector3<> P0, Vector3<> P1, Vector3<> P2, Vector3<> P3) {
  return P0 * std::pow((1 - t), 3) + P1 * 3 * std::pow((1 - t), 2) * t + P2 * 3 * (1 - t) * std::pow(t, 2) +
         P3 * std::pow(t, 3);
}

void generateStepTrajectoryRunSwift(WalkGeneratorData& generator,
                                    const Pose2D& leftGait,
                                    const Pose2D& rightGait,
                                    Vector3<>& leftFootOffset,
                                    Vector3<>& rightFootOffset,
                                    float& turnRL) {

  float swingRef = parabolicStep(generator.t, generator.stepDuration);
  float supportRef = Rangef::ZeroOneRange().limit(generator.t / generator.stepDuration);

  // 5.3.6 determine how high to lift the swing foot off the ground
  float swingHeight =
    generator.swingControlPoint.z * parabolicReturn(generator.t / generator.stepDuration); // lift swing foot
  float supportHeight =
    generator.swingControlPoint0.z * parabolicReturn((generator.switchPhase + generator.t) / generator.stepDuration);

  float leftRef, rightRef, leftHeight, rightHeight;
  if (generator.isLeftPhase) {
    leftRef = swingRef;
    leftHeight = swingHeight;
    rightHeight = supportHeight;
    rightRef = supportRef;

  } else {
    leftRef = supportRef;
    leftHeight = supportHeight;
    rightHeight = swingHeight;
    rightRef = swingRef;
  }
  Vector3<> leftGaitTarget = Vector3<>(leftGait.translation);
  Vector3<> rightGaitTarget = Vector3<>(rightGait.translation);
  float turnRLTarget = leftGait.rotation;
  if (generator.weightShiftStatus == WalkGeneratorData::weightDidShift) {
    leftFootOffset = generator.leftFootOffset0 + (leftGaitTarget - generator.leftFootOffset0) * leftRef;
    rightFootOffset = generator.rightFootOffset0 + (rightGaitTarget - generator.rightFootOffset0) * rightRef;
    turnRL = generator.turnRL0 + (turnRLTarget - generator.turnRL0) * supportRef;
  }
  leftFootOffset.z = leftHeight;
  rightFootOffset.z = rightHeight;
}

void generateStepTrajectoryBezier(WalkGeneratorData& generator,
                                  const Pose2D& leftGait,
                                  const Pose2D& rightGait,
                                  Vector3<>& leftFootOffset,
                                  Vector3<>& rightFootOffset,
                                  float& turnRL) {

  // 5.3.6 determine how high to lift the swing foot off the ground

  float t = Rangef::ZeroOneRange().limit(generator.t / generator.stepDuration);

  Vector3<> triggerOffset = Vector3<>(0.f, 0.f, -0.000f);
  Vector3<> leftGaitTarget;
  Vector3<> rightGaitTarget;
  float turnRLTarget = leftGait.rotation;
  if (generator.weightShiftStatus == WalkGeneratorData::weightDidShift) {
    leftGaitTarget = Vector3<>(leftGait.translation);
    rightGaitTarget = Vector3<>(rightGait.translation);
  } else {
    leftGaitTarget = generator.leftFootOffset0;
    rightGaitTarget = generator.rightFootOffset0;
  }

  if (generator.isLeftPhase) {

    Vector3<> swing0 = generator.leftFootOffset0;
    Vector3<> swing1 = (leftGaitTarget + generator.swingControlPoint * 2.f);
    Vector3<> swing2 = leftGaitTarget;

    Vector3<> support0 = generator.rightFootOffset0;
    Vector3<> support1 = generator.rightFootOffset0 + (rightGaitTarget - generator.rightFootOffset0) / 3.f;
    Vector3<> support2 =
      generator.rightFootOffset0 + (rightGaitTarget - generator.rightFootOffset0) * 2.f / 3.f + triggerOffset;
    Vector3<> support3 = (rightGaitTarget + triggerOffset);

    leftFootOffset = bezierQuadratic(t, swing0, swing1, swing2);
    rightFootOffset = bezierCubic(t, support0, support1, support2, support3);
    // rightFootOffset = support0 * (1 - t) + support3 * t;

  } else {
    Vector3<> swing0 = generator.rightFootOffset0;
    Vector3<> swing1 = (rightGaitTarget + generator.swingControlPoint * 2.f);
    Vector3<> swing2 = rightGaitTarget;

    Vector3<> support0 = generator.leftFootOffset0;
    Vector3<> support1 = generator.leftFootOffset0 + (leftGaitTarget - generator.leftFootOffset0) / 3.f;
    Vector3<> support2 =
      generator.leftFootOffset0 + (leftGaitTarget - generator.leftFootOffset0) * 2.f / 3.f + triggerOffset;
    Vector3<> support3 = (leftGaitTarget + triggerOffset);

    rightFootOffset = bezierQuadratic(t, swing0, swing1, swing2);
    leftFootOffset = bezierCubic(t, support0, support1, support2, support3);
    // leftFootOffset = support0 * (1 - t) + support3 * t;
  }
  turnRL = generator.turnRL0 + (turnRLTarget - generator.turnRL0) * t;
}
