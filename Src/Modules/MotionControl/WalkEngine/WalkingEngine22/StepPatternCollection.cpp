/**
 * @file StepPatternCollection.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "Core/Math/Vector.h"
#include "StepPatternCollection.h"
#include "Core/Debugging/DebugDrawings3D.h"

void StepPatternCollection::reset() {
  patternGenerated = false;
  isLeavingPossible = true;
  numberOfSteps = 0;
  specialPatternSteps.clear();
  targetSteps.clear();
  preSteps.clear();
}

void StepPatternCollection::update(PlannedSteps& plannedSteps, WalkGeneratorData& generator, InWalkKickRequest kickReq) {
  if (!specialPatternSteps.empty()) {
    specialPatternSteps.erase(specialPatternSteps.begin());
  }
  numberOfSteps = preSteps.size() + targetSteps.size();
  if (specialPatternSteps.empty()) {
    isLeavingPossible = true;
  } else {
    isLeavingPossible = false;
  }

  kickRequest = kickReq;
  if (!patternGenerated) {
    robotPoseAtBeginning = theRobotPose;
  }

  kickRequest.kickDirection = kickReq.kickDirection;

  if (theBallModel.estimate.position.y > 30) {
    leftFootAlignment = true;
  } else if (theBallModel.estimate.position.y < -30) {
    leftFootAlignment = false;
  } else {
    if (kickRequest.kickDirection < 0) {
      leftFootAlignment = true;
    } else {
      leftFootAlignment = false;
    }
  }
  if (!patternGenerated || specialPatternSteps.size() >= targetSteps.size()) {
    patternGenerated = true;
    generateGaitPatterns(plannedSteps);
    measuredPreviousOriginMotion = Pose2D();
    lastPatternTarget = theBallModel.estimate.position;
  } else {
    Leg::Side prevSupportSide = generator.isLeftPhase ? Leg::left : Leg::right;
    Pose2D measuredCurrentOrigin =
      generator.measuredCurrentStep[prevSupportSide] + (Pose2D() - generator.measuredInitialStep[prevSupportSide]);
    measuredPreviousOriginMotion = Pose2D() - measuredCurrentOrigin;
    for (Leg& step : specialPatternSteps) {
      step.legPose -= measuredPreviousOriginMotion;
    }
    lastPatternTarget -= measuredPreviousOriginMotion.translation;
  }
}

void StepPatternCollection::perStepGenerator(PlannedSteps& plannedSteps, Pose2D leftFoot, Pose2D rightFoot, Leg targetFoot) {
  Leg targetLeftFoot;
  Leg targetRightFoot;
  targetFoot.legPose.rotation = 0;
  if (targetFoot.side == Leg::left) {
    targetLeftFoot = targetFoot;
    targetRightFoot.side = Leg::right;
    targetRightFoot.legPose = targetFoot.legPose;
    targetRightFoot.legPose.translation += Vector2<>(0.f, -110.f);
  } else {
    targetRightFoot = targetFoot;
    targetLeftFoot.side = Leg::left;
    targetLeftFoot.legPose = targetFoot.legPose;
    targetLeftFoot.legPose.translation += Vector2<>(0.f, 110.f);
  }
  Pose2D leftDiff = targetLeftFoot.legPose.elementwiseSub(leftFoot);
  Pose2D rightDiff = targetRightFoot.legPose.elementwiseSub(rightFoot);
  Pose2D numOfMinStepsLeft = leftDiff.elementwiseDiv(maxStepSize);
  Pose2D numOfMinStepsRight = rightDiff.elementwiseDiv(maxStepSize);

  int minLeftStep2Target = std::round(-0.25f + std::max({std::abs(numOfMinStepsLeft.rotation),
                                                         std::abs(numOfMinStepsLeft.translation.x),
                                                         std::abs(numOfMinStepsLeft.translation.y)}));
  int minRightStep2Target = std::round(-0.25f + std::max({std::abs(numOfMinStepsRight.rotation),
                                                          std::abs(numOfMinStepsRight.translation.x),
                                                          std::abs(numOfMinStepsRight.translation.y)}));
  Leg startingStep;
  Leg targetStep;
  Vector2<> companionStepGaitOffset;
  int minStep2Target;

  startingStep.side = plannedSteps.currentExecutedStep.side;

  if (startingStep.side == Leg::left) {
    minStep2Target = minLeftStep2Target;
    startingStep.legPose = rightFoot;
    startingStep.legPose.translation += Vector2<>(0, 110);
    targetStep = targetLeftFoot;
    companionStepGaitOffset = Vector2<>(0, -110);
  } else {
    minStep2Target = minRightStep2Target;
    startingStep.legPose = leftFoot;
    startingStep.legPose.translation += Vector2<>(0, -110);
    targetStep = targetRightFoot;
    companionStepGaitOffset = Vector2<>(0, 110);
  }

  Leg step_i;

  for (int i = 1; i < minStep2Target; ++i) {
    // Main Leg
    step_i.side = startingStep.side;
    step_i.legPose = startingStep.legPose.scale(1.f - static_cast<float>(i) / (minStep2Target))
                       .elementwiseAdd(targetStep.legPose.scale(static_cast<float>(i) / (minStep2Target)));
    preSteps.push_back(step_i);
    // Companion Leg
    step_i.side = (startingStep.side == Leg::left) ? Leg::right : Leg::left;
    step_i.legPose = startingStep.legPose.scale(1.f - (static_cast<float>(i) + 0.5f) / (minStep2Target))
                       .elementwiseAdd(targetStep.legPose.scale((static_cast<float>(i) + 0.5f) / (minStep2Target)));
    step_i.legPose.translation.y = preSteps.back().legPose.translation.y;
    step_i.legPose.translation = step_i.legPose * companionStepGaitOffset;
    preSteps.push_back(step_i);
  }
  if (!preSteps.empty() && targetFoot.side == preSteps.back().side) {
    preSteps.pop_back();
  }
  if (preSteps.empty() && startingStep.side != targetFoot.side) {
    Leg auxillaryStep;
    Pose2D startingStepPose = targetFoot.side == Leg::left ? leftFoot : rightFoot;
    auxillaryStep.side = targetFoot.side == Leg::left ? Leg::right : Leg::left;
    auxillaryStep.legPose = startingStepPose.elementwiseAdd(targetFoot.legPose).scale(0.5f);
    auxillaryStep.legPose.translation = auxillaryStep.legPose * (-companionStepGaitOffset);
    preSteps.push_back(auxillaryStep);
  }
}

void StepPatternCollection::generatedefaultPatterns(PlannedSteps& plannedSteps) {
  Leg nextStep;
  float a[] = {0., 1., 2., 3., 4., 5., 6., 7., 8., 9.};
  Leg::Side currentSide;
  currentSide = Leg::left;
  if (currentSide == Leg::left) {
    for (float n : a) {
      nextStep.side = currentSide;
      if (currentSide == Leg::left) {
        nextStep.legPose = Pose2D(0.0f, 15.f + 40 * n, 55.f);
        currentSide = Leg::right;
      } else {
        nextStep.legPose = Pose2D(-0.0f, 15.f + 40 * n, -55.f);
        currentSide = Leg::left;
      }
      targetSteps.push_back(nextStep);
    }
  }
}

void StepPatternCollection::generatedefaultPatterns2(PlannedSteps& plannedSteps) {
  Leg nextStep;
  float a[] = {0., 1., 2., 3., 4., 5., 6., 7., 8., 9.};
  Leg::Side currentSide;
  currentSide = Leg::left;
  if (currentSide == Leg::left) {
    for (float n : a) {
      nextStep.side = currentSide;
      if (currentSide == Leg::left) {
        nextStep.legPose = Pose2D(0.4f, 15.f + 40 * n, 55.f);
        currentSide = Leg::right;
      } else {
        nextStep.legPose = Pose2D(-0.4f, 15.f + 40 * n, -55.f);
        currentSide = Leg::left;
      }
      targetSteps.push_back(nextStep);
    }
  }
}

void StepPatternCollection::generateStraightBallKickPatterns(PlannedSteps& plannedSteps) {
  Leg nextStep;

  Leg step0, step1, step2;
  step1.stepType = step0.stepType = Leg::prepare;
  step2.stepType = Leg::kick;
  Pose2D ballFrame = Pose2D(kickRequest.kickDirection, theBallModel.estimate.position);
  if (leftFootAlignment) {
    step1.side = Leg::right;
    step0.side = step2.side = Leg::left;
    step0.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-170, 0));
    step2.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-70, 0));
    step1.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-120, -100));

  } else {
    step1.side = Leg::left;
    step0.side = step2.side = Leg::right;

    step0.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-170, 0));
    step1.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-120, 100));
    step2.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-70, 0));
  }
  initialTargetPattern = step0;
  targetSteps.push_back(step0);
  targetSteps.push_back(step1);
  targetSteps.push_back(step2);
}

void StepPatternCollection::generateOmniBallKickPatterns(PlannedSteps& plannedSteps) {
  Leg nextStep;
  Leg step0, step1, step2;

  Pose2D ballFrame = Pose2D(kickRequest.kickDirection, theBallModel.estimate.position);
  step1.stepType = step0.stepType = Leg::prepare;
  step2.stepType = Leg::kick;

  if (leftFootAlignment) {
    step1.side = Leg::right;
    step0.side = step2.side = Leg::left;
    step2.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-60, -0));
    step0.legPose =
      step2.legPose + Pose2D(-kickRequest.kickDirection * 2 / 3.f, Vector2<>(-100.f, +kickRequest.kickDirection * 50));
    step1.legPose =
      step0.legPose + Pose2D(kickRequest.kickDirection / 3, Vector2<>(50, -110 + kickRequest.kickDirection * 20));
  } else {
    step1.side = Leg::left;
    step0.side = step2.side = Leg::right;
    step2.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-60, 0));
    step0.legPose =
      step2.legPose + Pose2D(-kickRequest.kickDirection * 2 / 3.f, Vector2<>(-100.f, kickRequest.kickDirection * 50));
    step1.legPose =
      step0.legPose + Pose2D(kickRequest.kickDirection * 1 / 3, Vector2<>(50, 110 + kickRequest.kickDirection * 20));
  }
  initialTargetPattern = step0;
  targetSteps.push_back(step0);
  targetSteps.push_back(step1);
  targetSteps.push_back(step2);
}

void StepPatternCollection::generateOmniBallKickPatterns2(PlannedSteps& plannedSteps) {
  Leg nextStep;
  Leg step0, step1, step2, step3;
  Vector2<> footOffset = Vector2<>(-60, 0);

  Pose2D ballFrame = Pose2D(kickRequest.kickDirection, theBallModel.estimate.position);
  step1.stepType = step0.stepType = Leg::prepare;
  step2.stepType = Leg::kick;
  bool localLeftFootAlignment = leftFootAlignment;
  if (kickRequest.kickDirection > fromDegrees(45.f)) {
    localLeftFootAlignment = true;
  } else if (kickRequest.kickDirection < fromDegrees(-45.f)) {
    localLeftFootAlignment = false;
  }
  if (localLeftFootAlignment) {
    step1.side = step3.side = Leg::right;
    step0.side = step2.side = Leg::left;
    step2.legPose = Pose2D(0.f, ballFrame.translation);
    step0.legPose = Pose2D(0.f, ballFrame * Vector2<>(-140.f, 0.f));
    step1.legPose = Pose2D(0.f,
                           Vector2<>((step0.legPose.translation.x + step2.legPose.translation.x) / 2,
                                     std::min(step0.legPose.translation.y, step2.legPose.translation.y) - 110));
    step3.legPose = Pose2D(0.f, step1.legPose.translation + (step2.legPose.translation - step0.legPose.translation) * 0.7);

  } else {
    step1.side = step3.side = Leg::left;
    step0.side = step2.side = Leg::right;

    step2.legPose = Pose2D(0.f, ballFrame.translation);

    step0.legPose = Pose2D(0.f, ballFrame * Vector2<>(-120.f, 0.f));
    step1.legPose = Pose2D(0.f,
                           Vector2<>((step0.legPose.translation.x + step2.legPose.translation.x) / 2,
                                     std::max(step0.legPose.translation.y, step2.legPose.translation.y) + 110));
    step3.legPose = Pose2D(0.f, step1.legPose.translation + (step2.legPose.translation - step0.legPose.translation) * 0.7);
    // Pose2D(0.f, ballFrame.translation + Vector2<>(step2.legPose.translation.x - step1.legPose.translation.x, 110.f));
  }
  step0.legPose.translation += footOffset;
  step1.legPose.translation += footOffset;
  step2.legPose.translation += footOffset;
  step3.legPose.translation += footOffset;
  initialTargetPattern = step0;
  targetSteps.push_back(step0);
  targetSteps.push_back(step1);
  targetSteps.push_back(step2);
  // targetSteps.push_back(step3);
}

void StepPatternCollection::generateAlignmentPatterns(PlannedSteps& plannedSteps) {
  Leg nextStep;
  Leg step0, step1, step2;

  Pose2D ballFrame = Pose2D(kickRequest.kickDirection, theBallModel.estimate.position);
  if (leftFootAlignment) {
    step1.side = Leg::left;
    step0.side = step2.side = Leg::right;
    step1.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-180, -10));
    step2.legPose = Pose2D(kickRequest.kickDirection, step1.legPose * Vector2<>(-0, -100));
    step0.legPose = Pose2D(kickRequest.kickDirection, step2.legPose.translation);
  } else {
    step1.side = Leg::right;
    step0.side = step2.side = Leg::left;
    step1.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-180, 10));
    step2.legPose = Pose2D(kickRequest.kickDirection, step1.legPose * Vector2<>(-0, 100));
    step0.legPose = Pose2D(kickRequest.kickDirection, step2.legPose.translation);
  }
  initialTargetPattern = step0;
  targetSteps.push_back(step0);
  targetSteps.push_back(step1);
  targetSteps.push_back(step2);
}

void StepPatternCollection::generateAccBallKickPatterns(PlannedSteps& plannedSteps) {
  Leg nextStep;

  Leg leftTarget0, rightTarget0;
  Leg leftTarget1, rightTarget1;
  Leg leftTarget2, rightTarget2;
  Leg leftTarget3;

  leftTarget0.side = leftTarget1.side = leftTarget2.side = leftTarget3.side = Leg::left;
  rightTarget0.side = rightTarget1.side = rightTarget2.side = Leg::right;
  Pose2D ballFrame = Pose2D(kickRequest.kickDirection, theBallModel.estimate.position);

  leftTarget0.legPose = Pose2D(kickRequest.kickDirection + pi_4, ballFrame * Vector2<>(-200, -50));
  rightTarget0.legPose = Pose2D(kickRequest.kickDirection + pi_4, ballFrame * Vector2<>(-120, -120));

  leftTarget1.legPose = Pose2D(kickRequest.kickDirection + pi_2, ballFrame * Vector2<>(-130, -40));
  rightTarget1.legPose = Pose2D(kickRequest.kickDirection, ballFrame * Vector2<>(-100, -120));
  leftTarget2.legPose = Pose2D(kickRequest.kickDirection + pi_2, ballFrame * Vector2<>(-60, -40));

  initialTargetPattern = leftTarget0;
  targetSteps.push_back(leftTarget0);
  targetSteps.push_back(rightTarget0);
  targetSteps.push_back(leftTarget1);
  targetSteps.push_back(rightTarget1);
  targetSteps.push_back(leftTarget2);
}

void StepPatternCollection::generateCirclePatterns(PlannedSteps& plannedSteps) {
  Leg nextStep;
  Leg leftIntNextStep;
  Leg rightIntNextStep;

  Leg tmpNextStep;

  Leg leftTarget0, rightTarget0;
  leftTarget0.side = Leg::left;
  rightTarget0.side = Leg::right;
  Pose2D ballFrame = Pose2D(kickRequest.kickDirection, theBallModel.estimate.position);
  Leg::Side currentSide;
  currentSide = Leg::left;
  float robot_ball_dist = sqrt(pow(theBallModel.estimate.position.x, 2) + pow(theBallModel.estimate.position.y, 2));
  float dist_of_circleAround = pi * robot_ball_dist;
  // float max_step_dist =  sqrt(pow(maxStepSize.translation[0],2)+pow(maxStepSize.translation[1],2));
  float max_step_dist = 60;
  int num_steps = round(dist_of_circleAround / max_step_dist);
  float ang_move = pi / num_steps;
  float x_step = max_step_dist * (1 - cos(ang_move));
  float y_step = max_step_dist * sin(ang_move);
  float ang_step = -atan2(x_step, y_step);
  Vector2<> leftcompanionStepGaitOffset;
  leftcompanionStepGaitOffset = Vector2<>(-12, 50);
  Vector2<> rightcompanionStepGaitOffset;
  rightcompanionStepGaitOffset = Vector2<>(-12, -50);

  // OUTPUT_TEXT(kickRequest.kickDirection<<" "<<x_step<<" "<<y_step<<" "<<ang_move);
  if (currentSide == Leg::left) {
    for (int i = 0; i < num_steps; ++i) {
      nextStep.side = currentSide;
      if (currentSide == Leg::left) {
        nextStep.legPose += Pose2D(--ang_move, x_step, y_step);
        tmpNextStep.legPose = Pose2D(nextStep.legPose * leftcompanionStepGaitOffset);
        currentSide = Leg::right;
        tmpNextStep.side = Leg::left;
        targetSteps.push_back(tmpNextStep);
      } else {
        // nextStep.legPose += Pose2D(-ang_move,x_step,y_step);
        tmpNextStep.side = Leg::right;
        tmpNextStep.legPose = Pose2D(nextStep.legPose * rightcompanionStepGaitOffset);
        currentSide = Leg::left;
        targetSteps.push_back(tmpNextStep);
      }
    }
  }
}

void StepPatternCollection::generateCirclePatterns2(PlannedSteps& plannedSteps) {
  Leg nextStep;
  Leg step0, step1, step2, step3, step4;
  const float maxCircleAngle = 0.7f;
  float circleAngle = Rangef(-maxCircleAngle, maxCircleAngle).clamp(kickRequest.kickDirection);
  Pose2D ballFrameEnd = Pose2D(circleAngle, theBallModel.estimate.position);
  Pose2D ballFrameBegin = Pose2D(0.f, theBallModel.estimate.position);
  const float circleRadius = 180.f;
  if (circleAngle > 0) {
    step0.side = step2.side = step4.side = Leg::left;
    step1.side = step3.side = Leg::right;

    step0.legPose = Pose2D(0.f, ballFrameBegin * Vector2<>(-circleRadius, 40));
    step4.legPose = Pose2D(circleAngle, ballFrameEnd * Vector2<>(-circleRadius, 40));
    step3.legPose = step4.legPose + Pose2D(0.f, 0, -60);
    step2.legPose = Pose2D(circleAngle, step0.legPose.translation) +
                    Pose2D(Vector2<>(circleRadius * (cos(circleAngle) - 1), -circleAngle * 20));
    step1.legPose = step0.legPose.elementwiseAdd(step2.legPose).scale(0.5) + Pose2D(0.f, 0, -100);
  } else {
    step0.side = step2.side = step4.side = Leg::right;
    step1.side = step3.side = Leg::left;
    step0.legPose = Pose2D(0.f, ballFrameBegin * Vector2<>(-circleRadius, -40));
    step4.legPose = Pose2D(circleAngle, ballFrameEnd * Vector2<>(-circleRadius, -40));
    step3.legPose = step4.legPose + Pose2D(0.f, 0, 60);
    step2.legPose = Pose2D(circleAngle, step0.legPose.translation) +
                    Pose2D(Vector2<>(circleRadius * (cos(circleAngle) - 1), -circleAngle * 20));
    step1.legPose = step0.legPose.elementwiseAdd(step2.legPose).scale(0.5) + Pose2D(0.f, 0, 100);
  }
  initialTargetPattern = step1;
  // targetSteps.push_back(step0);
  targetSteps.push_back(step1);
  targetSteps.push_back(step2);
  if (std::abs(circleAngle) >= maxCircleAngle) {
    targetSteps.push_back(step3);
    targetSteps.push_back(step4);
  }
}

void StepPatternCollection::generateCirclePatterns3(PlannedSteps& plannedSteps) {
  Leg nextStep;
  Leg step0, step1, step2;
  const float maxCircleAngle = 0.5f;
  float circleAngle = Rangef(-maxCircleAngle, maxCircleAngle).clamp(kickRequest.kickDirection);
  Pose2D ballFrameBegin = Pose2D(0.f, theBallModel.estimate.position);
  Pose2D ballFrameEnd = Pose2D(circleAngle, theBallModel.estimate.position);
  const float circleRadius = 200.f;
  if (circleAngle > 0) {
    step0.side = step2.side = Leg::left;
    step1.side = Leg::right;

    step0.legPose = Pose2D(0.f, ballFrameBegin * Vector2<>(-circleRadius, 50));
    step2.legPose = Pose2D(circleAngle * 3 / 2, ballFrameEnd * Vector2<>(-circleRadius, 50));
    step1.legPose = step2.legPose + Pose2D(-circleAngle / 2.f, 0, -100);
  } else {
    step0.side = step2.side = Leg::right;
    step1.side = Leg::left;

    step0.legPose = Pose2D(0.f, ballFrameBegin * Vector2<>(-circleRadius, -50));
    step2.legPose = Pose2D(circleAngle * 3 / 2, ballFrameEnd * Vector2<>(-circleRadius, -50));
    step1.legPose = step2.legPose + Pose2D(-circleAngle / 2.f, 0, 100);
  }
  initialTargetPattern = step1;
  // targetSteps.push_back(step0);
  targetSteps.push_back(step1);
  targetSteps.push_back(step2);
}

void StepPatternCollection::generateGaitPatterns(PlannedSteps& plannedSteps) {
  preSteps.clear();
  targetSteps.clear();
  switch (kickRequest.kickType) {
  case InWalkKickRequest::straightKick:
    generateStraightBallKickPatterns(plannedSteps);
    break;
  case InWalkKickRequest::accKick:
    generateAccBallKickPatterns(plannedSteps);
    break;
  case InWalkKickRequest::omniKick:
    generateOmniBallKickPatterns2(plannedSteps);
    // generateOmniBallKickPatterns(plannedSteps);
    break;
  case InWalkKickRequest::blindKick:
    generatedefaultPatterns(plannedSteps);
    break;
  case InWalkKickRequest::circleAround:
    // generateCirclePatterns(plannedSteps);
    generateCirclePatterns2(plannedSteps);
    // generateCirclePatterns3(plannedSteps);
    break;
  case InWalkKickRequest::alignment:
    generateAlignmentPatterns(plannedSteps);
    break;
  default:
    generatedefaultPatterns2(plannedSteps);
    break;
  }
  perStepGenerator(
    plannedSteps, plannedSteps.measuredStep[Leg::left], plannedSteps.measuredStep[Leg::right], initialTargetPattern);
  specialPatternSteps.clear();
  specialPatternSteps.insert(specialPatternSteps.end(), preSteps.begin(), preSteps.end());

  specialPatternSteps.insert(specialPatternSteps.end(), targetSteps.begin(), targetSteps.end());
}
