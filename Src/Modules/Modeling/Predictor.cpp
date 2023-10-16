/**
 * @file Predictor.cpp
 *
 * Implements a class that provides the robot pose as it will be after execution of the walking preview.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2023 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
 */

#include "Predictor.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/Watch.h"

/**
 * The method provides the robot pose after preview.
 * @param robotPoseAfterPreview The robot pose representation that is updated by this module.
 */
void Predictor::update(RobotPoseAfterPreview& robotPoseAfterPreview) {
  execute();

  (RobotPose&)robotPoseAfterPreview = theRobotPose;
  (Pose2D&)robotPoseAfterPreview += correctedOdometryOffset;

  PLOT("module:Predictor:robotPose.x", theRobotPose.translation.x);
  PLOT("module:Predictor:robotPose.y", theRobotPose.translation.y);
  PLOT("module:Predictor:robotPose.r", theRobotPose.rotation);
  PLOT("module:Predictor:robotPoseAfterPreview.x", robotPoseAfterPreview.translation.x);
  PLOT("module:Predictor:robotPoseAfterPreview.y", robotPoseAfterPreview.translation.y);
  PLOT("module:Predictor:robotPoseAfterPreview.r", robotPoseAfterPreview.rotation);

  WATCH(robotPoseAfterPreview.translation.x);
  WATCH(robotPoseAfterPreview.translation.y);
  WATCH(robotPoseAfterPreview.rotation);
}

/**
 * The method provides the ball model after preview.
 * @param ballModelAfterPreview The ball model representation that is updated by this module.
 */
void Predictor::update(BallModelAfterPreview& ballModelAfterPreview) {
  execute();
  (BallModel&)ballModelAfterPreview = theBallModel;

  Vector2<> position, velocity; // in global coordinates

  position = theRobotPose * theBallModel.estimate.position;
  velocity = theBallModel.estimate.velocity.rotated(theRobotPose.rotation);
  ballModelAfterPreview.estimate.position = positionAfterPreview.invert() * position;
  ballModelAfterPreview.estimate.velocity = velocity.rotated(positionAfterPreview.invert().rotation);

  position = theRobotPose * theBallModel.lastPerception;
  ballModelAfterPreview.lastPerception = positionAfterPreview.invert() * position;
}

void Predictor::execute() {
  if (lastExecutionTimeStamp < theFrameInfo.time) {
    correctedOdometryOffset = theMotionInfo.offsetToRobotPoseAfterPreview;
    positionAfterPreview = theRobotPose + correctedOdometryOffset;
    lastExecutionTimeStamp = theFrameInfo.time;
  }
}

MAKE_MODULE(Predictor, Modeling)
