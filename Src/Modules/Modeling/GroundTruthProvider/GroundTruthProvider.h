/**
 * @file GroundTruthProvider.h
 *
 * Declares & defines a class that provides ball & self location from SimRobot ground truth.
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GroundTruthWorldState.h"
#include "Representations/Modeling/PlayerModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/PenaltyMarkPercept.h"
#include "Representations/Perception/PlayerPercept.h"
#include "Core/Module/Module.h"

STREAMABLE(GroundTruthParameters,
           {
             ,
             (bool)disableBallValidity,
             (bool)disableRobotPoseValidity,
             (float)playerRadius,
             (float)playerHeight,
             (bool)lessResolution,
             (float)lineMaxRange,
             (float)lineMinLength,
             (float)penaltyMarkMaxRange,
           });

MODULE(GroundTruthProvider)
REQUIRES(CameraInfo)
REQUIRES(CameraMatrix)
REQUIRES(FieldDimensions)
REQUIRES(FrameInfo)
REQUIRES(GroundTruthWorldState)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthBallModel)
PROVIDES_WITH_MODIFY_AND_OUTPUT(GroundTruthPlayerModel)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthRobotPose)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallModel)
PROVIDES_WITH_MODIFY_AND_OUTPUT(LinePercept)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(PenaltyMarkPercept)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(PlayerModel)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(PlayerPercept)
PROVIDES_WITH_MODIFY_AND_OUTPUT(RobotPose)
LOADS_PARAMETER(GroundTruthParameters, theGroundTruthParams)
END_MODULE

/**
 * @class GroundTruthProvider
 * A GroundTruthProvider using some Kalman filters.
 */
class GroundTruthProvider : public GroundTruthProviderBase {
public:
  /**
   * Default constructor.
   */
  GroundTruthProvider() : lastGroundTruthTime(0) {}

  void update(GroundTruthBallModel& groundTruthBallModel);

  void update(GroundTruthPlayerModel& groundTruthPlayerModel);

  void update(GroundTruthRobotPose& groundTruthRobotPose);

  void update(BallModel& ballModel);

  void update(LinePercept& linePercept);

  void update(PenaltyMarkPercept& penaltyMarkPercept);

  void update(PlayerModel& playerModel);

  void update(PlayerPercept& playerPercept);

  void update(RobotPose& robotPose);

private:
  int lastGroundTruthTime;
  Vector2<> lastGlobalBallPos;
  GroundTruthBallModel groundTruthBallModel;
  GroundTruthPlayerModel groundTruthPlayerModel;
  GroundTruthRobotPose groundTruthRobotPose;

  void importGroundTruth();

  static void lowerResolution(Vector2<int>& p);
};
