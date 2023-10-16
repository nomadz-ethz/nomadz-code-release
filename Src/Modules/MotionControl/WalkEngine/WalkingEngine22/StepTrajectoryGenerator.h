/**
 * @file StepTrajectoryGenerator.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Representations/MotionControl/WalkGeneratorData.h"
#include "Representations/Sensing/FootSupport.h"
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
                                    const StepTraj::TrajType trajType) {

  float swingRef = parabolicStep(generator.t, generator.stepDuration);
  float supportRef = Rangef::ZeroOneRange().limit(generator.t / generator.stepDuration);

  // 5.3.6 determine how high to lift the swing foot off the ground
  float swingHeight =
    generator.swingControlPoint.z * parabolicReturn(generator.t / generator.stepDuration); // lift swing foot
  float supportHeight = 0 * parabolicReturn((generator.switchPhase + generator.t) / generator.stepDuration);

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
  generator.stepTraj[Leg::left].foot[trajType] =
    generator.stepTraj[Leg::left].foot[StepTraj::initialOffset] +
    (leftGaitTarget - generator.stepTraj[Leg::left].foot[StepTraj::initialOffset]) * leftRef;
  generator.stepTraj[Leg::right].foot[trajType] =
    generator.stepTraj[Leg::right].foot[StepTraj::initialOffset] +
    (rightGaitTarget - generator.stepTraj[Leg::right].foot[StepTraj::initialOffset]) * rightRef;

  generator.stepTraj[Leg::left].turnRL[trajType] =
    generator.stepTraj[Leg::left].turnRL[StepTraj::initialOffset] +
    (turnRLTarget - generator.stepTraj[Leg::left].turnRL[StepTraj::initialOffset]) * supportRef;
  generator.stepTraj[Leg::right].turnRL[trajType] = -generator.stepTraj[Leg::left].turnRL[StepTraj::currentOffset];

  generator.stepTraj[Leg::left].foot[trajType].z = leftHeight;
  generator.stepTraj[Leg::right].foot[trajType].z = rightHeight;
}

void generateStepTrajectoryBezier(WalkGeneratorData& generator,
                                  const Pose2D& leftGait,
                                  const Pose2D& rightGait,
                                  const StepTraj::TrajType trajType) {

  // 5.3.6 determine how high to lift the swing foot off the ground

  float t = Rangef::ZeroOneRange().limit(generator.t / generator.stepDuration);
  Vector3<> triggerOffset = Vector3<>(0.f, 0.f, generator.triggerStrength); // TriggerOffset not used for now
  Vector3<> gaitTarget[Leg::numOfSides];
  // Vector3<> rightGaitTarget;
  float turnRLTarget = leftGait.rotation;
  gaitTarget[Leg::left] = Vector3<>(leftGait.translation);
  gaitTarget[Leg::right] = Vector3<>(rightGait.translation);

  Leg::Side swingSide, supportSide;
  if (generator.isLeftPhase) {
    swingSide = Leg::left;
    supportSide = Leg::right;
  } else {
    swingSide = Leg::right;
    supportSide = Leg::left;
  }
  Vector3<> swing0 = generator.stepTraj[swingSide].foot[StepTraj::initialOffset];
  Vector3<> swing1 = swing0 + Vector3<>(0.f, 0.f, generator.swingControlPoint.z * 0.5);
  Vector3<> swing2 = (gaitTarget[swingSide] + generator.swingControlPoint * 2.0f);
  Vector3<> swing3 = gaitTarget[swingSide];

  Vector3<> support0 = generator.stepTraj[supportSide].foot[StepTraj::initialOffset];
  Vector3<> support3 = (gaitTarget[supportSide]);

  generator.stepTraj[swingSide].foot[trajType] = bezierQuadratic(t, swing0, swing2, swing3);
  // bezierCubic(t, swing0, swing1, swing2, swing3); // bezierQuadratic(t, swing0, swing2, swing3);
  //   ;
  generator.stepTraj[supportSide].foot[trajType] =
    support0 * (1 - t) + support3 * t; // bezierCubic(t, support0, support1, support2, support3);
  generator.stepTraj[Leg::left].turnRL[trajType] =
    generator.stepTraj[Leg::left].turnRL[StepTraj::initialOffset] * (1 - t) + turnRLTarget * t;

  generator.stepTraj[Leg::right].turnRL[trajType] = -generator.stepTraj[Leg::left].turnRL[trajType];
}

void generateStepTrajectory(WalkGeneratorData& generator,
                            const Pose2D& leftGait,
                            const Pose2D& rightGait,
                            const StepTraj::TrajType trajType,
                            const bool useBezier) {
  if (useBezier) {
    generateStepTrajectoryBezier(generator, leftGait, rightGait, trajType);
  } else {
    generateStepTrajectoryRunSwift(generator, leftGait, rightGait, trajType);
  }
}

void applyTriggerOffset(WalkGeneratorData& generator, Vector3<>& triggerFoot, float triggerStrength) {
  float uncroppedTriggerOffsetZ = -3.f * std::pow((generator.t / generator.stepDuration - 0.7), 2) + 0.2f;
  float triggerOffsetZ = 5 * std::max(0.f, uncroppedTriggerOffsetZ);
  triggerOffsetZ *= triggerStrength;
  triggerFoot += Vector3<>(0.f, 0.f, triggerOffsetZ);
}

void applyTriggerOffset2(WalkGeneratorData& generator, Vector3<>& triggerFoot, float triggerStrength) {
  float triggerOffsetZ =
    (std::max(0.f, (generator.t - generator.stepDuration + generator.triggerDuration)) / generator.triggerDuration) *
    triggerStrength;
  triggerFoot += Vector3<>(0.f, 0.f, triggerOffsetZ);
}

void linearInterpolate(WalkGeneratorData& generator, float startPhase, float endPhase) {
  float t = Rangef::ZeroOneRange().limit((generator.t - startPhase) / (generator.stepDuration * endPhase - startPhase));
  for (int i = 0; i < Leg::numOfSides; ++i) {
    generator.stepTraj[i].foot[StepTraj::combinedOffset] = generator.stepTraj[i].foot[StepTraj::currentOffset] * (1 - t) +
                                                           generator.stepTraj[i].foot[StepTraj::correctedOffset] * t;
    generator.stepTraj[i].turnRL[StepTraj::combinedOffset] =
      generator.stepTraj[i].turnRL[StepTraj::currentOffset] * (1 - t) +
      generator.stepTraj[i].turnRL[StepTraj::correctedOffset] * t;
  }
}

void generateOriginOffsetLinearInterpolate(
  WalkGeneratorData& generator, Vector3<> start, Vector3<> target, float startPhase, float endPhase) {
  float t = Rangef::ZeroOneRange().limit((generator.t - startPhase) / (endPhase - startPhase));
  generator.originOffset = start * (1 - t) + target * t;
}