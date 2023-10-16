/**
 * @file StepPlanner.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Representations/MotionControl/PlannedSteps.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/WalkGeneratorData.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/RobotDimensions.h"

#include "Core/Debugging/Debugging.h"
#include "Core/Module/Module.h"
#include "Core/Range.h"
#include "Core/Math/Vector.h"
#include "StepPatternCollection.h"

MODULE(StepPlanner)
USES(RobotPose)
REQUIRES(FrameInfo)
REQUIRES(RobotModel)
REQUIRES(RobotDimensions)
REQUIRES(MotionSelection)
REQUIRES(MotionRequest)
REQUIRES(BallModel)
PROVIDES_WITH_MODIFY(PlannedSteps)
LOADS_PARAMETER(Pose2D, maxSpeedLimit)
LOADS_PARAMETER(Pose2D, maxStepSize)
LOADS_PARAMETER(Vector2f, maxAcceleration)
LOADS_PARAMETER(Vector2f, maxDeceleration)
LOADS_PARAMETER(float, sideForwardMaxSpeed)
LOADS_PARAMETER(Pose2D, odometryScale)
LOADS_PARAMETER(float, targetModeSpeedFactor)
LOADS_PARAMETER(float, insideTurnRatio)
LOADS_PARAMETER(Pose2D, walkVolumeExponent)
LOADS_PARAMETER(float, baseWalkPeriod)
LOADS_PARAMETER(Vector2f, walkPeriodIncreaseFactor)
LOADS_PARAMETER(float, stepWidthOffset)
LOADS_PARAMETER(float, kickPrepareCompensationX)
LOADS_PARAMETER(float, kickPrepareCompensationY)
LOADS_PARAMETER(float, stepDurationIncreaseFactor)
LOADS_PARAMETER(float, kickDurationIncrease)
END_MODULE

class StepPlanner : public StepPlannerBase {
  const int minUpcomingPattern = 5;
  Pose2D currentPlannedSpeed;
  Pose2D lastSpeed;
  Pose2D deltaStep;
  Pose2D originAfterStep;

  Pose2D measuredPreviousOriginMotion;
  Pose2D globPosAtGaitSwitch;
  Pose2D robotPoseAtBeginning;

  Vector2<> offsetFromNorminal;
  Leg::Side swingSide;
  Leg::Side supportSide;
  float swingFootSign;

  float stepDuration;
  int numberOfAdditionalSteps;
  StepPatternCollection patternCollection;
  std::vector<Leg> previewSteps;

  void reset(PlannedSteps& plannedSteps);
  void update(PlannedSteps& plannedSteps) override;

  void calcStepPattern(PlannedSteps& plannedSteps,
                       WalkGeneratorData& generator,
                       Pose2D& requestedSpeed,
                       Pose2D& requestTarget,
                       WalkGeneratorData::WalkMode walkMode);
  void calcNextWayPoint(PlannedSteps& plannedSteps,
                        WalkGeneratorData& generator,
                        const Pose2D& speed,
                        const Pose2D& target,
                        WalkGeneratorData::WalkMode walkMode);
  void calcNextGaits(PlannedSteps& plannedSteps, WalkGeneratorData& generator, Pose2D& supportFootInitial, float& swingSign);
  void calcNextGaitFromPatterns(PlannedSteps& plannedSteps, WalkGeneratorData& generator, Pose2D& lFoot, Pose2D& rFoot);
  float calcPhaseDuration(WalkGeneratorData& generator);
  bool patternExecutionConditionFulfilled();
  bool ellipsoidClampWalk(Pose2D& curSpeed, Pose2D maxSpeed) const;
  bool rectClampWalk(Pose2D& current, Pose2D maxRef) const;
  float clampFootReq(Pose2D& foot, Pose2D maxRef, Leg::Side legSide);
  float calcWalkVolume(Pose2D curSpeed) const;
  float calcPhaseDuration(PlannedSteps& plannedSteps);
  void computeStepDurationForUpcomingSteps(PlannedSteps& plannedSteps);
  void customDebugDrawing(PlannedSteps& plannedSteps);
  void drawingGlobalStepPose(Pose2D stepPose, ColorRGBA color, int thicknessMultiplier = 1);
  Pose2D offset2Robot(WalkGeneratorData& generator, Pose2D offset, Leg::Side side);
  Pose2D robot2Offset(WalkGeneratorData& generator, Pose2D foot, Leg::Side side);

  bool insidePatternCone();
  Angle computeAlignmentAngle(const Vector2<>& relBallPos);

public:
  StepPlanner();
};
