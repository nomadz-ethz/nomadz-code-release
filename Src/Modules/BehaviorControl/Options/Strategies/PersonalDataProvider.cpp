/**
 * @file PersonalDataProvider.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "PersonalDataProvider.h"
#include "Core/Debugging/DebugDrawings3D.h"

MAKE_MODULE(PersonalDataProvider, Behavior Control);

PersonalDataProvider::PersonalDataProvider() {
  lastBallLockStateChangeTimeStamp = theFrameInfo.time;
}

void PersonalDataProvider::update(PersonalData& personalData) {
  DECLARE_DEBUG_DRAWING3D("module:PersonalDataProvider", "field");
  ballLockStateUpdate(personalData);
  personalData.shouldApproachBall = shouldApproachBall();
  personalData.hasBallLock = (personalData.ballLockState == PersonalData::HAS_LOCK);
  personalData.ballScore = ballScore;
  // draw();
}

void PersonalDataProvider::ballLockStateUpdate(PersonalData& personalData) {
  if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > hasBallLockLostTimeout ||
      (theGameInfo.state != STATE_PLAYING && theGameInfo.state != STATE_SET) || theRobotInfo.penalty != PENALTY_NONE) {
    if (personalData.ballLockState == PersonalData::HAS_LOCK) {
      personalData.syncTeamRequired = true;
    } else {
      personalData.syncTeamRequired = false;
    }
    personalData.ballLockState = PersonalData::NO_LOCK;
  }

  if (personalData.ballLockState != lastBallLockState) {
    newReqLockArrived = false;
    newHasLockArrived = false;
    // OUTPUT_TEXT(theRobotInfo.number << " " << personalData.ballLockState << " " << theFrameInfo.time);
    if (personalData.ballLockState == PersonalData::REQ_LOCK || personalData.ballLockState == PersonalData::HAS_LOCK ||
        lastBallLockState == PersonalData::HAS_LOCK) {
      personalData.syncTeamRequired = true;
    } else {
      personalData.syncTeamRequired = false;
    }
    lastBallLockStateChangeTimeStamp = theFrameInfo.time;
  } else {
    personalData.syncTeamRequired = false;
  }
  if (theTeamMateData.sendThisFrame) {
    lastSyncBallScore = ballScore;
    lastSyncTimeStamp = theFrameInfo.time;
  }
  ballScore = computeBallScore();
  if (personalData.ballLockState == PersonalData::HAS_LOCK) {
    if (theFrameInfo.time - lastSyncTimeStamp > 1000 && ballScore > lastSyncBallScore + 500.f) {
      personalData.syncTeamRequired = true;
    }
  }

  lastBallLockState = personalData.ballLockState;

  updateTeamMateBallLockState(personalData);
  switch (personalData.ballLockState) {
  case PersonalData::NO_LOCK:
    if (theFrameInfo.time - lastBallLockStateChangeTimeStamp > requestTimeout && theBallModel.valid &&
        ballScore < fixBallScoreThreshhold) {
      // if (anyTeamMateHasBallLock) {
      //   if (newHasLockArrived && ballScore < 0.9 * ballScoreFromHasLock && ballScore < ballScoreFromHasLock - 100) {
      //     personalData.ballLockState = PersonalData::REQ_LOCK;
      //   }
      // } else if (anyTeamMateReqBallLock) {
      //   if (newReqLockArrived && ballScore < bestBallScoreFromReqLock) {
      //     personalData.ballLockState = PersonalData::REQ_LOCK;
      //   }
      // } else if (!anyTeamMateHasBallLock && !anyTeamMateReqBallLock) {
      // }
      personalData.ballLockState = PersonalData::REQ_LOCK;
    }
    break;

  case PersonalData::REQ_LOCK:
    if (theFrameInfo.time - lastBallLockStateChangeTimeStamp > reqLockTimeout) {
      if (anyTeamMateHasBallLock) {
        personalData.ballLockState = PersonalData::NO_LOCK;
      } else if (anyTeamMateReqBallLock) {
        // TODO change the ballScore to the ballScore that was last sended to the teammate
        if (lastSyncBallScore < bestBallScoreFromReqLock) {
          personalData.ballLockState = PersonalData::HAS_LOCK;
        } else {
          personalData.ballLockState = PersonalData::NO_LOCK;
        }
      } else {
        personalData.ballLockState = PersonalData::HAS_LOCK;
      }
      resetTeamMateBallLockInfos(personalData);
    }
    break;

  case PersonalData::HAS_LOCK:
    if (newReqLockArrived && anyTeamMateReqBallLock) {
      if (ballScore > (bestBallScoreFromReqLock / 0.9) && ballScore > bestBallScoreFromReqLock + 100.f &&
          ballScore > 200.f) {
        personalData.ballLockState = PersonalData::NO_LOCK;
      }
    }

    break;

  default:
    break;
  }
}

void PersonalDataProvider::resetTeamMateBallLockInfos(PersonalData& personalData) {
  for (int i = 1; i < theTeamMateData.numOfPlayers; i++) {
    if (i == theRobotInfo.number) {
      continue;
    }
    personalData.teamMateBallLockStates[i] = PersonalData::NO_LOCK;
  }
}

void PersonalDataProvider::updateTeamMateBallLockState(PersonalData& personalData) {
  anyTeamMateHasBallLock = false;
  anyTeamMateReqBallLock = false;
  ballScoreFromHasLock = INFINITY;
  bestBallScoreFromReqLock = INFINITY;

  for (int i = 1; i < theTeamMateData.numOfPlayers; i++) {
    if (i == theRobotInfo.number) {
      continue;
    }
    if (theTeamMateData.timeStamps[i] > lastTeamMateTimeStamp[i]) {
      lastTeamMateTimeStamp[i] = theTeamMateData.timeStamps[i];
      personalData.teamMateBallLockStates[i] = theTeamMateData.robotsPersonalData[i].ballLockState;
      personalData.teamMateBallScores[i] = theTeamMateData.robotsPersonalData[i].ballScore;
    } else {
    }
    // Good but maybe not really needed.
    // if (theTeamMateData.timeStamps[i] - lastTeamMateTimeStamp[i] > theTeamMateData.networkTimeout) {
    //   teamMateBallLockInfos[i].ballLockState = PersonalData::NO_LOCK;
    // }
  }

  for (int i = 1; i < theTeamMateData.numOfPlayers; i++) {
    if (i == theRobotInfo.number) {
      continue;
    }
    if (theTeamMateData.isActive[i]) {
      if (personalData.teamMateBallLockStates[i] == PersonalData::HAS_LOCK) {
        anyTeamMateHasBallLock = true;
        ballScoreFromHasLock = theTeamMateData.robotsPersonalData[i].ballScore;
        if (theTeamMateData.timeStamps[i] > lastReceivedHasLockTimeStamp) {
          lastReceivedHasLockTimeStamp = theTeamMateData.timeStamps[i];
          newHasLockArrived = true;
        }
      }
      if (personalData.teamMateBallLockStates[i] == PersonalData::REQ_LOCK) {
        anyTeamMateReqBallLock = true;
        bestBallScoreFromReqLock = (theTeamMateData.robotsPersonalData[i].ballScore < bestBallScoreFromReqLock)
                                     ? static_cast<float>(theTeamMateData.robotsPersonalData[i].ballScore)
                                     : bestBallScoreFromReqLock;
        if (theTeamMateData.timeStamps[i] > lastReceivedReqLockTimeStamp) {
          lastReceivedReqLockTimeStamp = theTeamMateData.timeStamps[i];
          newReqLockArrived = true;
        }
      }
    }
  }
}

bool PersonalDataProvider::shouldApproachBall() {
  return theBallModel.valid && (!anyTeamMateHasBallLock || (ballScore < ballScoreFromHasLock));
}

float PersonalDataProvider::computeBallScore() {
  const float baseBallScore = theBallModel.estimate.position.abs();
  const float robotLostScoreModifier = theRobotPose.lost ? 1.1 : 1.0;
  const float circleAroundDistance = 0.f; // Disable this component for now
  const float keeperModifier = (theBehaviorStatus.role == BehaviorStatus::keeper) ? 1.3 : 1.f;
  const float ballLostScore = 4000.f;
  Angle circleAroundAngle = 0.f;

  if (!theRobotPose.lost) {
    Angle ball2Goal =
      (Vector2<>(theFieldDimensions.xPosOpponentGroundline, 0.f) - theRobotPose * theBallModel.estimate.position).angle();
    Angle robot2Ball = (theRobotPose * theBallModel.estimate.position - theRobotPose.translation).angle();
    circleAroundAngle = ball2Goal - robot2Ball;
    circleAroundAngle.normalize();
  } else {
    circleAroundAngle = pi;
  }
  float localBallScore = (baseBallScore * robotLostScoreModifier + circleAroundAngle * circleAroundDistance);
  if (theBallModel.valid) {
    ballScoreWhenLastValid = localBallScore;
  } else if (!theBallModel.lost) {
    const float t = (theBallModel.timeSinceLastSeen - 300) / (2000.f - 300.f);
    localBallScore = ballLostScore * t + ballScoreWhenLastValid * (1 - t);
  } else {
    localBallScore = ballLostScore;
  }
  return localBallScore;
}

void PersonalDataProvider::draw() {
  SPHERE3D("module:PersonalDataProvider",
           gloAlignmentTarget.translation.x,
           gloAlignmentTarget.translation.y,
           0,
           4,
           ColorClasses::red2);
}
