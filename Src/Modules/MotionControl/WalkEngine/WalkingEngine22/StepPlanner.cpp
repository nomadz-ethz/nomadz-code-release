/**
 * @file StepPlanner.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <algorithm>

#include "Core/Math/Rotation.h"

#include "StepPlanner.h"
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"
#include "Core/Math/Constants.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/ForwardKinematic.h"
#include "Core/Range.h"
#include "Core/Streams/InStreams.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"

#include "Representations/Sensing/RobotModel.h"

MAKE_MODULE(StepPlanner, Motion Control);

static const float mmPerM = 1000.f;

StepPlanner::StepPlanner() {}

void StepPlanner::reset(PlannedSteps& plannedSteps) {
  stepDuration = 0.f;
  currentSpeed = Pose2D();
  plannedSteps.nextLeftStep = Pose2D();
  plannedSteps.nextRightStep = Pose2D();
  // walkRequest.mode = WalkRequest::speedMode;
}

void StepPlanner::update(PlannedSteps& plannedSteps) {
  measuredLeftFoot = theRobotModel.limbs[MassCalibration::footLeft];
  measuredRightFoot = theRobotModel.limbs[MassCalibration::footRight];

  // Current Measured Position
  DECLARE_DEBUG_DRAWING3D("module:StepPlanner:plannedStep", "field");

  DECLARE_DEBUG_DRAWING3D("module:StepPlanner:nextStep", "robot");
  SPHERE3D_VEC("module:StepPlanner:nextStep",
               Vector3<>(measuredRightFoot.translation + Vector3<>(0.0f, 0.0f, -theRobotDimensions.heightLeg5Joint)),
               3,
               ColorClasses::purple1);
  SPHERE3D_VEC("module:StepPlanner:nextStep",
               Vector3<>(measuredLeftFoot.translation + Vector3<>(0.0f, 0.0f, -theRobotDimensions.heightLeg5Joint)),
               3,
               ColorClasses::purple1);

  SPHERE3D_VEC("module:StepPlanner:plannedStep", Vector3<>(globPosAtGaitSwitch.translation), 5, ColorClasses::purple1);
  plannedSteps.reset = [this, &plannedSteps]() { reset(plannedSteps); };
  plannedSteps.calcStepPattern =
    [this, &plannedSteps](const Pose2D& speed, const Pose2D& target, WalkGeneratorData::WalkMode walkMode) {
      calcStepPattern(plannedSteps, speed, target, walkMode);
    };
}

void StepPlanner::calcStepPattern(PlannedSteps& plannedSteps,
                                  const Pose2D& speed,
                                  const Pose2D& target,
                                  WalkGeneratorData::WalkMode walkMode) {
  plannedSteps.maxSpeed = maxSpeedLimit.elementwiseMul(odometryScale);

  plannedSteps.measuredLeftStep = Pose2D(measuredLeftFoot.rotation.getZAngle(), measuredLeftFoot.translation.toVec2());
  plannedSteps.measuredRightStep = Pose2D(measuredRightFoot.rotation.getZAngle(), measuredRightFoot.translation.toVec2());
  if (theWalkGeneratorData.t == 0) {
    globPosAtGaitSwitch = theRobotPose;

    if (walkMode != WalkGeneratorData::WalkMode::patternMode) {
      calcNextWayPoint(plannedSteps, speed, target, walkMode);
      calcNextGaits(plannedSteps, currentSpeed, plannedSteps.nextLeftStep, plannedSteps.nextRightStep);
    } else {
      generateSpecialGaitPatterns(plannedSteps, Vector2<>());
      calcNextGaitFromPatterns(plannedSteps, plannedSteps.nextLeftStep, plannedSteps.nextRightStep);
    }
    Pose2D deltaTarget = theWalkGeneratorData.isLeftPhase
                           ? plannedSteps.nextLeftStep - Pose2D(Vector3<>(theWalkGeneratorData.leftFootOffset0).toVec2())
                           : plannedSteps.nextRightStep - Pose2D(Vector3<>(theWalkGeneratorData.rightFootOffset0).toVec2());
    plannedSteps.stepDuration = stepDuration;
    plannedSteps.currentSpeed = deltaTarget.scale(0.5 / stepDuration);
    plannedSteps.maxFootHeight = calcStepHeight();
  }
}

void StepPlanner::generateSpecialGaitPatterns(PlannedSteps& plannedSteps, Vector2<> ballPosition) {
  UpcomingStep nextStep;
  if (theWalkGeneratorData.isLeftPhase) {
    nextStep.legSide = UpcomingStep::left;
    nextStep.legPose = Pose2D(0.1f, 30.f, 185.f);
    nextStep.stepDuration = 0.3f;
  } else {
    nextStep.legSide = UpcomingStep::right;
    nextStep.legPose = Pose2D(-0.1f, 30.f, -185.f);
    nextStep.stepDuration = 0.3f;
  }
  plannedSteps.upcomingSteps.push_back(nextStep);
  // plannedSteps.currentSpeed = Pose2D(0.f, 0.01f, 0.f);
  // plannedSteps.maxFootHeight = calcStepHeight();
}

void StepPlanner::calcNextGaitFromPatterns(PlannedSteps& plannedSteps, Pose2D& lFoot, Pose2D& rFoot) {
  currentExecutedStep = plannedSteps.upcomingSteps.front();
  plannedSteps.upcomingSteps.erase(plannedSteps.upcomingSteps.begin());
  ASSERT(currentExecutedStep.legSide == (theWalkGeneratorData.isLeftPhase ? UpcomingStep::left : UpcomingStep::right));
  Pose2D deltaStep;
  plannedSteps.stepDuration = currentExecutedStep.stepDuration;
  if (theWalkGeneratorData.isLeftPhase) {
    deltaStep = currentExecutedStep.legPose - Pose2D(Vector3<>(theWalkGeneratorData.leftRefPoint).toVec2()) -
                Pose2D(-theWalkGeneratorData.turnRL0, Vector3<>(theWalkGeneratorData.leftFootOffset0).toVec2() * mmPerM);
    lFoot = deltaStep.scale(1.f / 2);
    rFoot = deltaStep.scale(-1.f / 2);
  } else {
    deltaStep = currentExecutedStep.legPose - Pose2D(Vector3<>(theWalkGeneratorData.rightRefPoint).toVec2()) -
                Pose2D(theWalkGeneratorData.turnRL0, Vector3<>(theWalkGeneratorData.rightFootOffset0).toVec2() * mmPerM);
    lFoot = deltaStep.scale(-1.f / 2);
    rFoot = deltaStep.scale(1.f / 2);
  }
  lFoot.translation /= mmPerM;
  rFoot.translation /= mmPerM;
}

void StepPlanner::calcNextGaits(PlannedSteps& plannedSteps, Pose2D& currentSpeed, Pose2D& lFoot, Pose2D& rFoot) {
  Vector2<> support, swing;
  float swingFootSign;
  if (theWalkGeneratorData.isLeftPhase) {
    swingFootSign = -1;
  } else {
    swingFootSign = 1;
  }

  support.x = -currentSpeed.translation.x / 2.f;
  swing.x = currentSpeed.translation.x / 2.f;

  support.y = currentSpeed.translation.y * swingFootSign < 0 ? -currentSpeed.translation.y : 0.f;
  swing.y = currentSpeed.translation.y * swingFootSign < 0 ? currentSpeed.translation.y : 0.f;

  turnRL = (currentSpeed.rotation * swingFootSign < 0 ? 1.f - insideTurnRatio : insideTurnRatio) * -swingFootSign *
           currentSpeed.rotation;

  if (theWalkGeneratorData.isLeftPhase) {
    lFoot = Pose2D(turnRL, swing);
    rFoot = Pose2D(-turnRL, support);
  } else {
    lFoot = Pose2D(turnRL, support);
    rFoot = Pose2D(-turnRL, swing);
  }
  lFoot.translation.y += stepWidthOffset * std::abs(currentSpeed.translation.x) / 278;
  rFoot.translation.y -= stepWidthOffset * std::abs(currentSpeed.translation.x) / 278;
}

void StepPlanner::calcNextWayPoint(PlannedSteps& plannedSteps,
                                   const Pose2D& speed,
                                   const Pose2D& target,
                                   WalkGeneratorData::WalkMode walkMode) {
  Pose2D request = speed;
  maxSpeed = maxSpeedLimit;
  if (walkMode == WalkGeneratorData::targetMode) {
    ASSERT(speed.rotation > 0.f && speed.translation.x > 0.f && speed.translation.y > 0.f);
    maxSpeed = Pose2D(std::min(speed.rotation / odometryScale.rotation, maxSpeed.rotation),
                      std::min(speed.translation.x / odometryScale.translation.x, maxSpeed.translation.x),
                      std::min(speed.translation.y / odometryScale.translation.y, maxSpeed.translation.y));
    sideForwardMaxSpeed = std::min(
      speed.translation.x / odometryScale.translation.x / maxSpeed.translation.x * sideForwardMaxSpeed, sideForwardMaxSpeed);
    maxSpeedBackwards = std::min(speed.translation.x / odometryScale.translation.x, maxSpeedBackwards);
    // Remove the offset that will be covered just by returning the swing leg
    currentSpeed = (target - theWalkGeneratorData.returnOffset).elementwiseDiv(odometryScale);
    // If the leg swings in target direction, consider that the next step will move the robot the same distance again
    if (currentSpeed.translation.y * (theWalkGeneratorData.isLeftPhase ? 1.f : -1.f) > 0) {
      currentSpeed.translation.y *= 0.5f;
    }
    stepDuration = calcPhaseDuration(plannedSteps);
    plannedSteps.speed = currentSpeed.scale(1 / stepDuration);
    if (ellipsoidClampWalk(plannedSteps.speed)) {
      walkMode = WalkGeneratorData::speedMode;
      request = target.scale(targetModeSpeedFactor); // targetModeSpeedFactor = 2, why we double the speed ?
    } else {
      // Consider in the speed that half of the step is returning to origin
      float turnFactor = currentSpeed.rotation * (theWalkGeneratorData.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f - insideTurnRatio
                                                                                                     : insideTurnRatio;
      float leftFactor = currentSpeed.translation.y * (theWalkGeneratorData.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f : 0.f;
      plannedSteps.speed =
        plannedSteps.speed.elementwiseMul(Pose2D(turnFactor, 0.5f, leftFactor)).elementwiseMul(odometryScale) +
        theWalkGeneratorData.returnOffset.scale(1 / stepDuration);

      plannedSteps.upcomingOdometryOffset =
        theWalkGeneratorData.returnOffset + plannedSteps.speed.elementwiseMul(Pose2D(turnFactor, 0.5f, leftFactor))
                                              .elementwiseMul(odometryScale)
                                              .scale(stepDuration * 2);
    }
  }

  if (walkMode == WalkGeneratorData::speedMode) {
    currentSpeed = request.elementwiseDiv(odometryScale);
    ellipsoidClampWalk(currentSpeed);

    // If switching direction, first stop if new speed is not reachable through acceleration
    if (lastSpeed.translation.x * currentSpeed.translation.x < 0.f &&
        std::abs(lastSpeed.translation.x) > std::abs(maxAcceleration.x)) {
      currentSpeed.translation.x = 0.f;
    }

    // Limit acceleration and deceleration of forward movement
    if (lastSpeed.translation.x > 0.f || (lastSpeed.translation.x == 0.f && currentSpeed.translation.x > 0.f)) {
      currentSpeed.translation.x =
        lastSpeed.translation.x +
        Rangef(-maxDeceleration.x, maxAcceleration.x).limit(currentSpeed.translation.x - lastSpeed.translation.x);
    } else {
      currentSpeed.translation.x =
        lastSpeed.translation.x +
        Rangef(-maxAcceleration.x, maxDeceleration.x).limit(currentSpeed.translation.x - lastSpeed.translation.x);
    }

    // If switching direction, first stop if new speed is not reachable through acceleration
    if (lastSpeed.translation.y * currentSpeed.translation.y < 0.f &&
        std::abs(lastSpeed.translation.y) > std::abs(maxAcceleration.y)) {
      currentSpeed.translation.y = 0.f;
    }

    // Limit acceleration and deceleration of sideways movement
    if (lastSpeed.translation.y > 0.f || (lastSpeed.translation.y == 0.f && currentSpeed.translation.y > 0.f)) {
      currentSpeed.translation.y =
        lastSpeed.translation.y +
        Rangef(-maxDeceleration.y, maxAcceleration.y).limit(currentSpeed.translation.y - lastSpeed.translation.y);
    } else {
      currentSpeed.translation.y =
        lastSpeed.translation.y +
        Rangef(-maxAcceleration.y, maxDeceleration.y).limit(currentSpeed.translation.y - lastSpeed.translation.y);
    }

    stepDuration = calcPhaseDuration(plannedSteps);

    // Consider in the speed that half of the step is returning to origin
    float turnFactor =
      currentSpeed.rotation * (theWalkGeneratorData.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f - insideTurnRatio : insideTurnRatio;
    float leftFactor = currentSpeed.translation.y * (theWalkGeneratorData.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f : 0.f;

    plannedSteps.speed = currentSpeed.elementwiseMul(Pose2D(turnFactor, 0.5f, leftFactor)).elementwiseMul(odometryScale) +
                         theWalkGeneratorData.returnOffset.scale(1 / stepDuration);

    plannedSteps.upcomingOdometryOffset = currentSpeed.elementwiseMul(Pose2D(turnFactor, 0.5f, leftFactor))
                                            .elementwiseMul(odometryScale)
                                            .scale(stepDuration * 2) +
                                          theWalkGeneratorData.returnOffset;
  }

  if (walkMode == WalkGeneratorData::speedMode) {
    lastSpeed.translation = currentSpeed.translation;

    // 1.6 Walk Calibration
    // The definition of forward, left and turn is the actual distance/angle traveled in one second.
    // It is scaled down to the duration of a single step.
    currentSpeed = currentSpeed.scale(stepDuration);
  } else {
    lastSpeed.translation = currentSpeed.translation / stepDuration;
  }
  currentSpeed.translation /= mmPerM;
}

float StepPlanner::calcWalkVolume(Pose2D curSpeed) const {
  float forward = std::abs(curSpeed.translation.x);
  float left = std::abs(curSpeed.translation.y);
  float turn = std::abs(curSpeed.rotation);
  return std::pow(std::pow(forward, walkVolumeExponent.translation.x) + std::pow(left, walkVolumeExponent.translation.y),
                  (walkVolumeExponent.rotation / walkVolumeExponent.translation.y)) +
         std::pow(turn, walkVolumeExponent.rotation);
}

bool StepPlanner::ellipsoidClampWalk(Pose2D& curSpeed) const {
  // Values in range [-1..1]
  Pose2D maxSpeedRef = maxSpeed;
  maxSpeedRef.translation.x = (curSpeed.translation.x >= 0.f ? maxSpeed.translation.x : maxSpeedBackwards);
  Pose2D currentSpeedAmount = curSpeed.elementwiseDiv(maxSpeedRef).scale(1 / maxSpeedDivider);

  float factor = std::max(currentSpeedAmount.translation.max(), std::abs(currentSpeedAmount.rotation));
  bool clamp = factor > 1.f;
  if (clamp) {
    currentSpeedAmount.translation /= factor;
  }
  // Clip based on a lower max turn speed. This causes a stronger clipping for forward and left, but allows for fast turns.
  // Otherwise the robot might fall more often.
  // float clampTurnAmount = std::min(std::abs(turn) / useMaxTurnSpeedForClampWalk, 1.f);

  if (currentSpeedAmount.rotation >= 0) {
    currentSpeedAmount.rotation = std::min(currentSpeedAmount.rotation, 1.f); // useMaxTurnSpeedForClampWalk
  } else {
    currentSpeedAmount.rotation = std::max(currentSpeedAmount.rotation, -1.f);
  }

  // see if the point we are given is already inside the allowed walk params volume
  if (calcWalkVolume(currentSpeedAmount) > 1.f) {
    clamp = true;
    float scale = 0.5f;
    float high = 1.f;
    float low = 0.f;

    // This is basically a binary search to find the point on the surface.
    for (unsigned i = 0; i < 10; ++i) {
      // give priority to turn. keep it the same
      Pose2D scaledCurrentSpeedAmount = Pose2D(currentSpeedAmount.rotation, currentSpeedAmount.translation * scale);
      if (calcWalkVolume(scaledCurrentSpeedAmount) > 1.f) {
        high = scale;
      } else {
        low = scale;
      }
      scale = (low + high) / 2.f;
    }

    currentSpeedAmount.translation *= scale;
  }
  curSpeed = maxSpeed;
  curSpeed.translation.x = (curSpeed.translation.x >= 0.f ? maxSpeed.translation.x : maxSpeedBackwards);
  curSpeed = curSpeed.elementwiseMul(currentSpeedAmount).scale(maxSpeedDivider);
  return clamp;
}

float StepPlanner::calcPhaseDuration(PlannedSteps& plannedSteps) {
  return (baseWalkPeriod + walkPeriodIncreaseFactor.x * currentSpeed.translation.x / plannedSteps.maxSpeed.translation.x +
          walkPeriodIncreaseFactor.y * std::abs(currentSpeed.translation.y)) /
         mmPerM;
}

/* float StepPlanner::calcPhaseDurationFromStepSize(Pose2D& dTarget) {
  return (baseWalkPeriod + walkPeriodIncreaseFactor.x * currentSpeed.translation.x / plannedSteps.maxSpeed.translation.x +
          walkPeriodIncreaseFactor.y * std::abs(currentSpeed.translation.y)) /
         1000.f;
}

float StepPlanner::calcStepHeightFromStepSize(Pose2D& dTarget, float duration) {
  return baseFootLift / mmPerM + std::abs(currentSpeed.translation.x) * footLiftIncreaseFactor.x +
         std::abs(currentSpeed.translation.y) * footLiftIncreaseFactor.y;
} */

float StepPlanner::calcStepHeight() {
  return baseFootLift / mmPerM + std::abs(currentSpeed.translation.x) * footLiftIncreaseFactor.x +
         std::abs(currentSpeed.translation.y) * footLiftIncreaseFactor.y;
}
