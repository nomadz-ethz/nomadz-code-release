/**
 * @file Predictor.h
 *
 * Declares a class that predicts RobotPose etc. based on the knowledge
 * how we have moved after execution of the current motion (ie. preview while walking).
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2023 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"

MODULE(Predictor)
REQUIRES(FrameInfo)
REQUIRES(MotionInfo)
REQUIRES(RobotPose)
REQUIRES(BallModel)
PROVIDES_WITH_MODIFY(RobotPoseAfterPreview)
PROVIDES_WITH_MODIFY(BallModelAfterPreview)
END_MODULE

/**
 * @class PosePredictor
 */
class Predictor : public PredictorBase {
public:
  /** Default constructor. */
  Predictor() {
    correctedOdometryOffset = Pose2D();
    positionAfterPreview = Pose2D();
    lastExecutionTimeStamp = 0;
  }

  /** Destructor */
  ~Predictor() {}

  /**
   * The method provides the robot pose after preview.
   * @param robotPoseAfterPreview The robot pose representation that is updated by this module.
   */
  void update(RobotPoseAfterPreview& robotPoseAfterPreview);

  /**
   * The method provides the ball model after preview.
   * @param ballModelAfterPreview The ball model representation that is updated by this module.
   */
  void update(BallModelAfterPreview& ballModelAfterPreview);

  /*
   * calculate the corrected odometry offset for preview only once
   */
  void execute();

  Pose2D correctedOdometryOffset;
  Pose2D positionAfterPreview;
  unsigned lastExecutionTimeStamp;
};
