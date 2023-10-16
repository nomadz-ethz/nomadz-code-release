/**
 * @file PersonalDataProvider.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/PersonalData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/MotionControl/KickEngineOutput.h"

#include "Core/Debugging/Debugging.h"
#include "Core/Module/Module.h"
#include "Core/Range.h"
#include "Core/Math/Vector.h"

struct TeamMateBallLockInfo {
  int robotNumber;
  char ballLockState;
  float ballScore;

  TeamMateBallLockInfo() {
    robotNumber = 0;
    ballLockState = PersonalData::NO_LOCK;
    ballScore = INFINITY;
  }

  TeamMateBallLockInfo(int number, char lockState, float score) {
    robotNumber = number;
    ballLockState = lockState;
    ballScore = score;
  };
};

MODULE(PersonalDataProvider)
USES(KickEngineOutput)
REQUIRES(FrameInfo)
REQUIRES(BallModel)
REQUIRES(OwnTeamInfo)
REQUIRES(RobotInfo)
REQUIRES(TeamMateData)
USES(TeamDataSenderOutput)
REQUIRES(RobotPose)
REQUIRES(GameInfo)
REQUIRES(BehaviorStatus)
REQUIRES(FieldDimensions)
REQUIRES(CombinedWorldModel)
PROVIDES_WITH_MODIFY(PersonalData)
END_MODULE

class PersonalDataProvider : public PersonalDataProviderBase {

  void reset(PersonalData& personalData);
  void update(PersonalData& personalData) override;
  void ballLockStateUpdate(PersonalData& personalData);
  bool computeWantsBall();
  float computeExtraScore();
  bool hasBallLock(bool wantsBall, float extraScore);
  Vector2<> getCombinedBallPosition();
  bool ballInGoalBox(Vector2<> gloBall);
  float computeBallScore();
  void updateTeamMateBallLockState(PersonalData& personalData);
  void resetTeamMateBallLockInfos(PersonalData& personalData);
  bool shouldApproachBall();
  void draw();

private:
  const float fixBallScoreThreshhold = 1000;
  const unsigned reqLockTimeout = 500;
  const unsigned hasBallLockLostTimeout = 6000;
  const unsigned requestTimeout = 2000;
  bool wantsBall;
  float ballScore = INFINITY;
  float lastSyncBallScore;
  unsigned lastSyncTimeStamp;
  bool sameBallDetected;
  Pose2D gloAlignmentTarget;
  float ballScoreWhenLastValid;
  TeamMateData lastTeamSenderOutput;
  unsigned lastBallLockStateChangeTimeStamp;
  unsigned lastReceivedHasLockTimeStamp;
  unsigned lastReceivedReqLockTimeStamp;
  unsigned lastTeamMateTimeStamp[8];
  bool newHasLockArrived;
  bool newReqLockArrived;
  char lastBallLockState;
  bool anyTeamMateHasBallLock;
  bool anyTeamMateReqBallLock;
  float ballScoreFromHasLock;
  float bestBallScoreFromReqLock;
  // TeamMateBallLockInfo teamMateBallLockInfos[8];

public:
  PersonalDataProvider();
};
