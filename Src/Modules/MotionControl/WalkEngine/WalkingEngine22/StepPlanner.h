/**
 * @file StepPlanner.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once
#include "Representations/MotionControl/PlannedSteps.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/WalkGeneratorData.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/RobotDimensions.h"

#include "Core/Debugging/Debugging.h"
#include "Core/Module/Module.h"
#include "Core/Range.h"
#include "Core/Math/Vector.h"

MODULE(StepPlanner)
USES(RobotPose)
REQUIRES(FrameInfo)
REQUIRES(RobotModel)
USES(WalkGeneratorData)
REQUIRES(RobotDimensions)
REQUIRES(MotionSelection)
PROVIDES_WITH_MODIFY(PlannedSteps)
LOADS_PARAMETER(Pose2D, maxSpeedLimit)
LOADS_PARAMETER(Vector2f, maxAcceleration)
LOADS_PARAMETER(Vector2f, maxDeceleration)
LOADS_PARAMETER(float, maxSpeedBackwards)
LOADS_PARAMETER(float, maxSpeedDivider)
LOADS_PARAMETER(float, sideForwardMaxSpeed)
LOADS_PARAMETER(Pose2D, odometryScale)
LOADS_PARAMETER(float, targetModeSpeedFactor)
LOADS_PARAMETER(float, insideTurnRatio)
LOADS_PARAMETER(Pose2D, walkVolumeExponent)
LOADS_PARAMETER(float, baseWalkPeriod)
LOADS_PARAMETER(Vector2f, walkPeriodIncreaseFactor)
LOADS_PARAMETER(float, stepWidthOffset)
LOADS_PARAMETER(float, baseFootLift)
LOADS_PARAMETER(Vector2f, footLiftIncreaseFactor)
END_MODULE

class StepPlanner : public StepPlannerBase {

  enum WalkState { standing, starting, walking, stopping } walkState; /**< The current state of the engine. */
  Pose2D currentSpeed;
  Pose2D lastSpeed;

  Pose3D leftFoot;
  Pose3D rightFoot;

  Pose3D measuredLeftFoot;
  Pose3D measuredRightFoot;
  Pose2D maxSpeed;
  Angle turnRL;
  Angle turnRL0;
  UpcomingStep currentExecutedStep;
  float stepDuration;

  // Debugging Drawing related variables
  std::vector<Vector3<>> leftTraj;
  std::vector<Vector3<>> rightTraj;

  Pose2D globPosAtGaitSwitch;

  void reset(PlannedSteps& plannedSteps);
  void update(PlannedSteps& plannedSteps);
  void calcStepPattern(PlannedSteps& plannedSteps,
                       const Pose2D& speed,
                       const Pose2D& target,
                       WalkGeneratorData::WalkMode walkMode);
  void generateSpecialGaitPatterns(PlannedSteps& plannedSteps, Vector2<> ballPosition);
  void calcNextWayPoint(PlannedSteps& plannedSteps,
                        const Pose2D& speed,
                        const Pose2D& target,
                        WalkGeneratorData::WalkMode walkMode);
  void calcNextGaitFromPatterns(PlannedSteps& plannedSteps, Pose2D& lFoot, Pose2D& rFoot);
  void calcNextGaits(PlannedSteps& plannedSteps, Pose2D& currentSpeed, Pose2D& lFoot, Pose2D& rFoot);
  float calcPhaseDuration(WalkGeneratorData& generator);
  bool ellipsoidClampWalk(Pose2D& curSpeed) const;
  float calcWalkVolume(Pose2D curSpeed) const;
  float calcPhaseDuration(PlannedSteps& plannedSteps);
  float calcStepHeight();
  Pose2D gaitOrigin2RobotPose(Pose2D gaitOrigin);
  Pose2D gaitRobotPose2Origin(Pose2D gaitRobotPose);

public:
  StepPlanner();
};
