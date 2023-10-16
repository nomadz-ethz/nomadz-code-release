/**
 * @file StepPatternCollection.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Math/Vector.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/PlannedSteps.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Representations/Sensing/FootSupport.h"

class StepPatternCollection {
public:
  StepPatternCollection(const BallModel& theBallModel, const RobotPose& theRobotPose, Pose2D maxStepSize)
      : theBallModel(theBallModel), theRobotPose(theRobotPose), maxStepSize(maxStepSize){};
  void reset();
  void update(PlannedSteps& plannedSteps, WalkGeneratorData& generator, InWalkKickRequest kickReq);
  void perStepGenerator(PlannedSteps& plannedSteps, Pose2D leftFoot, Pose2D rightFoot, Leg targetFoot);
  void generatedefaultPatterns(PlannedSteps& plannedSteps);
  void generatedefaultPatterns2(PlannedSteps& plannedSteps);
  void generateStraightBallKickPatterns(PlannedSteps& plannedSteps);
  void generateAlignmentPatterns(PlannedSteps& plannedSteps);
  void generateOmniBallKickPatterns(PlannedSteps& plannedSteps);
  void generateOmniBallKickPatterns2(PlannedSteps& plannedSteps);
  void generateAccBallKickPatterns(PlannedSteps& plannedSteps);
  void generateGaitPatterns(PlannedSteps& plannedSteps);
  void generateCirclePatterns(PlannedSteps& plannedSteps);
  void generateCirclePatterns2(PlannedSteps& plannedSteps);
  void generateCirclePatterns3(PlannedSteps& plannedSteps);

private:
  std::vector<Leg> preSteps;
  Pose2D maxStepSize;
  Leg initialTargetPattern;
  const BallModel& theBallModel;
  const RobotPose& theRobotPose;
  bool leftFootAlignment;

public:
  Vector2<> lastPatternTarget;
  std::vector<Leg> targetSteps;
  bool patternGenerated;
  int numberOfSteps;
  bool isLeavingPossible;
  Pose2D measuredPreviousOriginMotion;
  Pose2D robotPoseAtBeginning;
  float currentKickDirection;
  InWalkKickRequest kickRequest;
  std::vector<Leg> specialPatternSteps;
};
