/**
 * @file StepPlanner.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
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

StepPlanner::StepPlanner() : patternCollection(theBallModel, theRobotPose, maxStepSize) {}

void StepPlanner::reset(PlannedSteps& plannedSteps) {
  stepDuration = 0.f;
  currentPlannedSpeed = Pose2D();
  for (int i = 0; i < Leg::numOfSides; ++i) {
    plannedSteps.nextStep[i] = Pose2D();
  }
  // walkRequest.mode = WalkRequest::speedMode;
  patternCollection.reset();
}

void StepPlanner::update(PlannedSteps& plannedSteps) {
  DECLARE_DEBUG_DRAWING3D("module:StepPlanner:plannedStep", "field");
  customDebugDrawing(plannedSteps);
  plannedSteps.reset = [this, &plannedSteps]() { reset(plannedSteps); };
  plannedSteps.calcStepPattern =
    [this,
     &plannedSteps](WalkGeneratorData& generator, Pose2D& speed, Pose2D& target, WalkGeneratorData::WalkMode walkMode) {
      calcStepPattern(plannedSteps, generator, speed, target, walkMode);
    };
}

void StepPlanner::calcStepPattern(PlannedSteps& plannedSteps,
                                  WalkGeneratorData& generator,
                                  Pose2D& requestedSpeed,
                                  Pose2D& requestedTarget,
                                  WalkGeneratorData::WalkMode walkMode) {
  swingSide = (generator.isLeftPhase) ? Leg::left : Leg::right;
  supportSide = (generator.isLeftPhase) ? Leg::right : Leg::left;
  swingFootSign = generator.isLeftPhase ? 1.f : -1.f;

  plannedSteps.prevExecutedStep = Leg(plannedSteps.currentExecutedStep);
  plannedSteps.currentExecutedStep.side = swingSide;
  plannedSteps.measuredStep[Leg::left] = generator.measuredCurrentStep[Leg::left];
  plannedSteps.measuredStep[Leg::right] = generator.measuredCurrentStep[Leg::right];
  offsetFromNorminal =
    (plannedSteps.measuredStep[Leg::left].translation + plannedSteps.measuredStep[Leg::right].translation) / 2.f -
    Vector2<>(-12.f, 0.f);
  for (int i = 0; i < Leg::numOfSides; ++i) {
    plannedSteps.measuredStep[i].translation -= offsetFromNorminal;
  }
  plannedSteps.maxSpeed = maxSpeedLimit.elementwiseMul(odometryScale);
  globPosAtGaitSwitch = theRobotPose;
  globPosAtGaitSwitch.translation = globPosAtGaitSwitch * Vector2<>(12.f, 0.f);

  calcNextWayPoint(plannedSteps, generator, requestedSpeed, requestedTarget, walkMode);
  InWalkKickRequest inWalkKickRequest = theMotionRequest.walkRequest.inWalkKickRequest;
  if ((walkMode == WalkGeneratorData::WalkMode::patternMode || !patternCollection.isLeavingPossible) &&
      patternExecutionConditionFulfilled()) {
    patternCollection.update(plannedSteps, generator, inWalkKickRequest);
    deltaStep = plannedSteps.currentPlannedSpeed.scale(stepDuration);
  } else {
    patternCollection.lastPatternTarget = theBallModel.estimate.position;
    patternCollection.reset();
  }
  if (patternCollection.specialPatternSteps.empty() ||
      ((theBallModel.estimate.position - patternCollection.lastPatternTarget).abs() > 100)) {
    patternCollection.reset();
    // Immediately follow up with other pattern
    if (walkMode == WalkGeneratorData::WalkMode::patternMode &&
        inWalkKickRequest.kickType == InWalkKickRequest::circleAround) {
      patternCollection.update(plannedSteps, generator, inWalkKickRequest);
      deltaStep = plannedSteps.currentPlannedSpeed.scale(stepDuration);
    }
  }

  calcNextGaits(plannedSteps, generator, plannedSteps.measuredStep[supportSide], swingFootSign);
  plannedSteps.upcomingSteps.clear();
  plannedSteps.upcomingSteps.insert(plannedSteps.upcomingSteps.end(),
                                    patternCollection.specialPatternSteps.begin(),
                                    patternCollection.specialPatternSteps.end());

  plannedSteps.upcomingSteps.insert(plannedSteps.upcomingSteps.end(), previewSteps.begin(), previewSteps.end());

  computeStepDurationForUpcomingSteps(plannedSteps);

  calcNextGaitFromPatterns(plannedSteps, generator, plannedSteps.nextStep[swingSide], plannedSteps.nextStep[supportSide]);
  plannedSteps.isCurrentStepKick = plannedSteps.currentExecutedStep.stepType == Leg::kick;
  plannedSteps.isCurrentStepPrepare = plannedSteps.currentExecutedStep.stepType == Leg::prepare;

  clampFootReq(plannedSteps.nextStep[Leg::left], maxSpeedLimit.scale(2 * generator.stepDuration), Leg::left);
  clampFootReq(plannedSteps.nextStep[Leg::right], maxSpeedLimit.scale(2 * generator.stepDuration), Leg::right);

  if (plannedSteps.currentExecutedStep.stepType == Leg::prepare) {
    plannedSteps.nextStep[swingSide].translation.x -= kickPrepareCompensationX * inWalkKickRequest.kickStrength;
    plannedSteps.nextStep[supportSide].translation.x -= kickPrepareCompensationX * inWalkKickRequest.kickStrength;
  }
  if (plannedSteps.currentExecutedStep.stepType == Leg::kick) {
    plannedSteps.nextStep[swingSide].translation.y +=
      swingFootSign * kickPrepareCompensationY * inWalkKickRequest.kickStrength;
    plannedSteps.nextStep[supportSide].translation.y +=
      swingFootSign * kickPrepareCompensationY * inWalkKickRequest.kickStrength;
  }

  originAfterStep = plannedSteps.measuredStep[supportSide] +
                    (Pose2D() - offset2Robot(generator,
                                             Pose2D(plannedSteps.nextStep[supportSide]),
                                             supportSide)); // The relative origin motion covered by the next step
  originAfterStep = (originAfterStep == Pose2D()) ? Pose2D(0.f, 0.01f, 0.f) : originAfterStep;

  plannedSteps.isLeavingPossible = patternCollection.isLeavingPossible;
  plannedSteps.measuredStep[supportSide] + (offset2Robot(generator, plannedSteps.nextStep[swingSide], swingSide) -
                                            offset2Robot(generator, plannedSteps.nextStep[supportSide], supportSide));

  plannedSteps.stepDuration = plannedSteps.currentExecutedStep.stepDuration;
  plannedSteps.currentPlannedSpeed =
    (walkMode == WalkGeneratorData::patternMode) ? originAfterStep.scale(1.f / stepDuration) : plannedSteps.speed;
}

bool StepPlanner::patternExecutionConditionFulfilled() {
  // if (patternCollection.specialPatternSteps.size() <= patternCollection.targetSteps.size()) {
  //   return theBallModel.valid;
  // } else {
  return (insidePatternCone() && !theBallModel.lost) ||
         (std::abs(theMotionRequest.walkRequest.inWalkKickRequest.kickDirection) > fromDegrees(60));
  // }
  // return !theBallModel.lost && theBallModel.estimate.position.abs() < 600 &&
  //        theBallModel.estimate.position.angle() < fromDegrees(35);
}

bool StepPlanner::insidePatternCone() {
  const float patternAlignmentDistance = 350.f;
  const float patternAlignmentAngle = fromDegrees(25.f);
  if (theBallModel.estimate.position.abs() < patternAlignmentDistance) {
    return std::abs(computeAlignmentAngle(theBallModel.estimate.position)) < patternAlignmentAngle;
  } else
    return false;
}

Angle StepPlanner::computeAlignmentAngle(const Vector2<>& relBallPos) {
  const float alignmentOffsetY = 75.f;
  if (relBallPos.y > alignmentOffsetY) {
    return Vector2<>(relBallPos.x, relBallPos.y - alignmentOffsetY).angle();
  } else if (relBallPos.y < -alignmentOffsetY) {
    return Vector2<>(relBallPos.x, relBallPos.y + alignmentOffsetY).angle();
  } else {
    return Angle(0.f);
  }
}

void StepPlanner::calcNextWayPoint(PlannedSteps& plannedSteps,
                                   WalkGeneratorData& generator,
                                   const Pose2D& speed,
                                   const Pose2D& target,
                                   WalkGeneratorData::WalkMode walkMode) {
  Pose2D request = speed;
  Pose2D maxSpeed = maxSpeedLimit;
  if (walkMode == WalkGeneratorData::targetMode) {
    ASSERT(speed.rotation > 0.f && speed.translation.x > 0.f && speed.translation.y > 0.f);
    maxSpeed = Pose2D(std::min(speed.rotation / odometryScale.rotation, maxSpeed.rotation),
                      std::min(speed.translation.x / odometryScale.translation.x, maxSpeed.translation.x),
                      std::min(speed.translation.y / odometryScale.translation.y, maxSpeed.translation.y));
    sideForwardMaxSpeed = std::min(
      speed.translation.x / odometryScale.translation.x / maxSpeed.translation.x * sideForwardMaxSpeed, sideForwardMaxSpeed);
    // Remove the offset that will be covered just by returning the swing leg
    Leg::Side previousSupportSide = swingSide;
    deltaStep = (target - robot2Offset(generator, plannedSteps.measuredStep[previousSupportSide], previousSupportSide))
                  .elementwiseDiv(odometryScale);

    stepDuration = calcPhaseDuration(plannedSteps);
    plannedSteps.speed = deltaStep.scale(1 / stepDuration);
    if (ellipsoidClampWalk(plannedSteps.speed, maxSpeed)) {
      walkMode = WalkGeneratorData::speedMode;
      request = target.scale(targetModeSpeedFactor); // targetModeSpeedFactor = 2, why we double the speed ?
    }
  }

  if (walkMode == WalkGeneratorData::speedMode || walkMode == WalkGeneratorData::patternMode) {
    currentPlannedSpeed = request.elementwiseDiv(odometryScale);
    ellipsoidClampWalk(currentPlannedSpeed, maxSpeed);

    // If switching direction, first stop if new speed is not reachable through acceleration
    if (lastSpeed.translation.x * currentPlannedSpeed.translation.x < 0.f &&
        std::abs(lastSpeed.translation.x) > std::abs(maxAcceleration.x)) {
      currentPlannedSpeed.translation.x = 0.f;
    }

    // Limit acceleration and deceleration of forward movement
    if (lastSpeed.translation.x > 0.f || (lastSpeed.translation.x == 0.f && currentPlannedSpeed.translation.x > 0.f)) {
      currentPlannedSpeed.translation.x =
        lastSpeed.translation.x +
        Rangef(-maxDeceleration.x, maxAcceleration.x).limit(currentPlannedSpeed.translation.x - lastSpeed.translation.x);
    } else {
      currentPlannedSpeed.translation.x =
        lastSpeed.translation.x +
        Rangef(-maxAcceleration.x, maxDeceleration.x).limit(currentPlannedSpeed.translation.x - lastSpeed.translation.x);
    }

    // If switching direction, first stop if new speed is not reachable through acceleration
    if (lastSpeed.translation.y * currentPlannedSpeed.translation.y < 0.f &&
        std::abs(lastSpeed.translation.y) > std::abs(maxAcceleration.y)) {
      currentPlannedSpeed.translation.y = 0.f;
    }

    // Limit acceleration and deceleration of sideways movement
    if (lastSpeed.translation.y > 0.f || (lastSpeed.translation.y == 0.f && currentPlannedSpeed.translation.y > 0.f)) {
      currentPlannedSpeed.translation.y =
        lastSpeed.translation.y +
        Rangef(-maxDeceleration.y, maxAcceleration.y).limit(currentPlannedSpeed.translation.y - lastSpeed.translation.y);
    } else {
      currentPlannedSpeed.translation.y =
        lastSpeed.translation.y +
        Rangef(-maxAcceleration.y, maxDeceleration.y).limit(currentPlannedSpeed.translation.y - lastSpeed.translation.y);
    }
    stepDuration = calcPhaseDuration(plannedSteps);
  }

  lastSpeed.translation = currentPlannedSpeed.translation;
  if (walkMode == WalkGeneratorData::speedMode || walkMode == WalkGeneratorData::patternMode) {
    deltaStep = currentPlannedSpeed.scale(stepDuration);
    plannedSteps.speed = currentPlannedSpeed;
  }
  float turnFactor = deltaStep.rotation * swingFootSign > 0 ? 1.f - insideTurnRatio : insideTurnRatio;
  plannedSteps.speed = plannedSteps.speed.elementwiseMul(Pose2D(turnFactor, 1.f, 1.f)).elementwiseMul(odometryScale);

  plannedSteps.upcomingOdometryOffset = plannedSteps.speed.scale(stepDuration);
}

void StepPlanner::calcNextGaits(PlannedSteps& plannedSteps,
                                WalkGeneratorData& generator,
                                Pose2D& supportFoot,
                                float& swingSign) {
  Vector2<> support, swing;
  float swingSign_i;
  Leg::Side swingSide_i;
  Leg::Side supportSide_i;
  Pose2D supportFootInitial;

  previewSteps.clear();
  if (!patternCollection.specialPatternSteps.empty()) {
    swingSign_i = (patternCollection.specialPatternSteps.back().side == Leg::left) ? -1.f : 1.f;
    swingSide_i = (patternCollection.specialPatternSteps.back().side == Leg::left) ? Leg::right : Leg::left;
    supportSide_i = patternCollection.specialPatternSteps.back().side;
    supportFootInitial = patternCollection.specialPatternSteps.back().legPose;
  } else {
    swingSign_i = swingSign;
    swingSide_i = swingSide;
    supportSide_i = supportSide;
    supportFootInitial = supportFoot;
  }
  numberOfAdditionalSteps = minUpcomingPattern - patternCollection.specialPatternSteps.size();
  for (int i = 0; i < numberOfAdditionalSteps; ++i) {
    Angle turnRL;
    Leg step_i;

    support.y = deltaStep.translation.y * swingSign_i > 0.f ? -deltaStep.translation.y : 0.f;
    swing.y = deltaStep.translation.y * swingSign_i > 0.f ? deltaStep.translation.y : 0.f;

    turnRL =
      (deltaStep.rotation * swingSign_i > 0.f ? 1.f - insideTurnRatio : insideTurnRatio) * swingSign_i * deltaStep.rotation;

    support.x = -deltaStep.translation.x / 2.f;
    swing.x = deltaStep.translation.x / 2.f;
    // Using Delta Step to calculate the next steps
    step_i.side = swingSide_i;

    Pose2D swingFoot = Pose2D(swingSign_i * turnRL, swing);
    Pose2D supportFoot = Pose2D(-swingSign_i * turnRL, support);

    step_i.legPose = supportFootInitial +
                     (offset2Robot(generator, swingFoot, swingSide_i) - offset2Robot(generator, supportFoot, supportSide_i));
    previewSteps.push_back(step_i);
    swingSide_i = (swingSide_i == Leg::left) ? Leg::right : Leg::left;
    supportSide_i = (supportSide_i == Leg::left) ? Leg::right : Leg::left;
    swingSign_i = (swingSign_i == 1.f) ? -1.f : 1.f;
    supportFootInitial = previewSteps.back().legPose;
  }
}

void StepPlanner::calcNextGaitFromPatterns(PlannedSteps& plannedSteps,
                                           WalkGeneratorData& generator,
                                           Pose2D& swingFoot,
                                           Pose2D& supportFoot) {

  plannedSteps.currentExecutedStep = plannedSteps.upcomingSteps.front();

  Pose2D nextOrigin =
    plannedSteps.currentExecutedStep.legPose.elementwiseAdd(plannedSteps.measuredStep[supportSide]).scale(1.f / 2);
  nextOrigin.translation = nextOrigin * Vector2<>(12.f, 0.f);
  swingFoot = robot2Offset(generator, (plannedSteps.currentExecutedStep.legPose - nextOrigin), swingSide);
  supportFoot = robot2Offset(generator, (plannedSteps.measuredStep[supportSide] - nextOrigin), supportSide);
}

float StepPlanner::calcWalkVolume(Pose2D curRatio) const {
  float forward = std::abs(curRatio.translation.x);
  float left = std::abs(curRatio.translation.y);
  float turn = std::abs(curRatio.rotation);
  return std::pow(std::pow(forward, walkVolumeExponent.translation.x) + std::pow(left, walkVolumeExponent.translation.y),
                  (walkVolumeExponent.rotation / walkVolumeExponent.translation.y)) +
         std::pow(turn, walkVolumeExponent.rotation);
}

bool StepPlanner::ellipsoidClampWalk(Pose2D& current, Pose2D maxRef) const {
  // Values in range [-1..1]
  Pose2D currentRatio = current.elementwiseDiv(maxRef);

  float factor = std::max(currentRatio.translation.max(), std::abs(currentRatio.rotation));
  bool clamp = factor > 1.f;
  if (clamp) {
    currentRatio.translation /= factor;
  }
  currentRatio.rotation = Rangef(-1, 1).clamp(currentRatio.rotation);

  // see if the point we are given is already inside the allowed walk params volume
  if (calcWalkVolume(currentRatio) > 1.f) {
    clamp = true;
    float scale = 0.5f;
    float high = 1.f;
    float low = 0.f;

    // This is basically a binary search to find the point on the surface.
    for (unsigned i = 0; i < 10; ++i) {
      // give priority to turn. keep it the same
      Pose2D scaledCurrentSpeedAmount = Pose2D(currentRatio.rotation, currentRatio.translation * scale);
      if (calcWalkVolume(scaledCurrentSpeedAmount) > 1.f) {
        high = scale;
      } else {
        low = scale;
      }
      scale = (low + high) / 2.f;
    }

    currentRatio.translation *= scale;
  }
  current = maxRef.elementwiseMul(currentRatio);
  return clamp;
}

bool StepPlanner::rectClampWalk(Pose2D& current, Pose2D maxRef) const {
  bool clamp = (std::abs(current.translation.x) > maxRef.translation.x) ||
               (std::abs(current.translation.y) > maxRef.translation.y) || (std::abs(current.rotation) > maxRef.rotation);
  current.translation.x = Rangef(-maxRef.translation.x, maxRef.translation.x).clamp(current.translation.x);
  current.translation.y = Rangef(-maxRef.translation.y, maxRef.translation.y).clamp(current.translation.y);
  current.rotation = Rangef(-maxRef.rotation, maxRef.rotation).clamp(current.rotation);
  if (clamp) {
    OUTPUT_TEXT("Invalid Step Requested");
  }
  return clamp;
}

float StepPlanner::clampFootReq(Pose2D& foot, Pose2D maxRef, Leg::Side legSide) {
  Rangef validRegionX(-maxRef.scale(1.f / 2).translation.x, maxRef.scale(1.f / 2).translation.x);
  Rangef validRegionY = (legSide == Leg::left) ? Rangef(-5.f, maxRef.scale(1.f / 2).translation.y)
                                               : Rangef(-maxRef.scale(1.f / 2).translation.y, 5.f);
  Rangef validRegionR = (legSide == Leg::left)
                          ? Rangef(-1.f / 3 * maxRef.scale(1.f / 2).rotation, 2.f / 3 * maxRef.scale(1.f / 2).rotation)
                          : Rangef(-2.f / 3 * maxRef.scale(1.f / 2).rotation, 1.f / 3 * maxRef.scale(1.f / 2).rotation);
  bool clamp = !validRegionX.contains(foot.translation.x) || !validRegionY.contains(foot.translation.y) ||
               !validRegionR.contains(foot.rotation);
  foot.translation.x = validRegionX.clamp(foot.translation.x);
  foot.translation.y = validRegionY.clamp(foot.translation.y);
  foot.rotation = validRegionR.clamp(foot.rotation);
  if (clamp) {
    // OUTPUT_TEXT("Invalid Step Requested " << legSide);
  }
  return clamp;
}

float StepPlanner::calcPhaseDuration(PlannedSteps& plannedSteps) {
  return (baseWalkPeriod +
          walkPeriodIncreaseFactor.x * currentPlannedSpeed.translation.x / plannedSteps.maxSpeed.translation.x +
          walkPeriodIncreaseFactor.y * std::abs(currentPlannedSpeed.translation.y));
}

void StepPlanner::computeStepDurationForUpcomingSteps(PlannedSteps& plannedSteps) {
  Pose2D step_i = plannedSteps.measuredStep[swingSide];
  for (Leg& upcomingStep : plannedSteps.upcomingSteps) {
    Pose2D strideSize = upcomingStep.legPose.elementwiseSub(step_i);
    upcomingStep.stepDuration = baseWalkPeriod * (1.f + stepDurationIncreaseFactor * strideSize.translation.abs() /
                                                          maxSpeedLimit.scale(0.25f).translation.abs());
    if (upcomingStep.stepType == Leg::prepare) {
      upcomingStep.stepDuration *= kickDurationIncrease;
    }
    step_i = upcomingStep.legPose;
  }
}

void StepPlanner::customDebugDrawing(PlannedSteps& plannedSteps) {

  SPHERE3D_VEC("module:StepPlanner:plannedStep", Vector3<>(globPosAtGaitSwitch.translation), 2, ColorClasses::blue2);
  Vector3<> directionPoint = Vector3<>(globPosAtGaitSwitch * Vector2<>(50.f, 0.f));
  LINE3D_VEC(
    "module:StepPlanner:plannedStep", Vector3<>(globPosAtGaitSwitch.translation), directionPoint, 2, ColorClasses::blue2);
  Vector3<> directionPointy =
    Vector3<>(theRobotPose *
              (theBallModel.estimate.position + Pose2D(patternCollection.kickRequest.kickDirection) * Vector2<>(50.f, 0.f)));
  Vector3<> ballPosGlo = Vector3<>(theRobotPose * theBallModel.estimate.position);
  LINE3D_VEC("module:StepPlanner:plannedStep", ballPosGlo, directionPointy, 10, ColorClasses::blue2);
  drawingGlobalStepPose(plannedSteps.currentExecutedStep.legPose, ColorClasses::red2, 2);
  for (size_t i = 1; i < plannedSteps.upcomingSteps.size(); i++) {
    drawingGlobalStepPose(
      plannedSteps.upcomingSteps[i].legPose + Pose2D(0.f, 0.f, 0),
      ColorRGBA(238,
                130,
                238,
                (int)(255 * (2 * plannedSteps.upcomingSteps.size() + 1 - i) / (1 + 2 * plannedSteps.upcomingSteps.size()))));
  }
}

Pose2D StepPlanner::offset2Robot(WalkGeneratorData& generator, Pose2D offset, Leg::Side side) {
  return offset.elementwiseAdd(Pose2D(generator.stepTraj[side].foot[StepTraj::refPoint].toVec2()));
}

Pose2D StepPlanner::robot2Offset(WalkGeneratorData& generator, Pose2D foot, Leg::Side side) {
  return foot.elementwiseSub(Pose2D(generator.stepTraj[side].foot[StepTraj::refPoint].toVec2()));
}

void StepPlanner::drawingGlobalStepPose(Pose2D stepPose, ColorRGBA color, int thicknessMultiplier) {
  Vector2<> cornerLF_rel(theRobotDimensions.footFront, theRobotDimensions.footOuter);
  Vector2<> cornerRF_rel(theRobotDimensions.footFront, -theRobotDimensions.footInner);
  Vector2<> cornerLB_rel(-theRobotDimensions.footBack, theRobotDimensions.footOuter);
  Vector2<> cornerRB_rel(-theRobotDimensions.footBack, -theRobotDimensions.footInner);

  Vector3<> cornerLF = Vector3<>(globPosAtGaitSwitch * (stepPose * cornerLF_rel));
  Vector3<> cornerRF = Vector3<>(globPosAtGaitSwitch * (stepPose * cornerRF_rel));
  Vector3<> cornerLB = Vector3<>(globPosAtGaitSwitch * (stepPose * cornerLB_rel));
  Vector3<> cornerRB = Vector3<>(globPosAtGaitSwitch * (stepPose * cornerRB_rel));
  LINE3D_VEC("module:StepPlanner:plannedStep", cornerRF, cornerLF, 2 * thicknessMultiplier, color);
  LINE3D_VEC("module:StepPlanner:plannedStep", cornerLF, cornerLB, 1 * thicknessMultiplier, color);
  LINE3D_VEC("module:StepPlanner:plannedStep", cornerLB, cornerRB, 2 * thicknessMultiplier, color);
  LINE3D_VEC("module:StepPlanner:plannedStep", cornerRB, cornerRF, 1 * thicknessMultiplier, color);

  // SPHERE3D_VEC("module:StepPlanner:plannedStep", Vector3<>(globPosAtGaitSwitch * stepPose.translation), 4, color);
}