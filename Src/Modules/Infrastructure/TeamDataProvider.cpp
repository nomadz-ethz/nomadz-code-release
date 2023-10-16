/**
 * @file TeamDataProvider.cpp
 *
 * This file implements a module that provides the data received by team communication.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TeamDataProvider.h"
#include "Core/Settings.h"
#include "Core/System/Time.h"
#include "Tools/Team.h"

/**
 * This macro unpacks compressed representations. It reads
 * representationCompressed from the MessageQueue and unpacks it into
 * teamMateData.array[robotNumber].
 */
#define UNPACK(representation, array)                                                                                       \
  representation##Compressed the##representation##Compressed;                                                               \
  message.bin >> the##representation##Compressed;                                                                           \
  theTeamMateData.array[robotNumber] = the##representation##Compressed;

/**
 * This macro converts a timeStamp into local time via ntp.
 */
#define REMOTE_TO_LOCAL_TIME(timeStamp)                                                                                     \
  if (timeStamp) {                                                                                                          \
    int ts;                                                                                                                 \
    ts = ntp.getRemoteTimeInLocalTime(timeStamp);                                                                           \
    if (ts < 0)                                                                                                             \
      timeStamp = 0;                                                                                                        \
    else                                                                                                                    \
      timeStamp = ts;                                                                                                       \
  }

PROCESS_WIDE_STORAGE(TeamDataProvider) TeamDataProvider::theInstance = 0;

TeamDataProvider::TeamDataProvider() : timeStamp(0), robotNumber(-1), lastSentTimeStamp(0), initialDataSent(false) {
  theInstance = this;
  lastNtpMessageSyncTime = theFrameInfo.time;
}

TeamDataProvider::~TeamDataProvider() {
  theInstance = 0;
}

void TeamDataProvider::update(TeamMateData& teamMateData) {
  if (ntp.localId == 255) {
    // NTP does not use a unique ID yet, seed it
    ntp = NTP(theRobotInfo.number);
  }

  DECLARE_PLOT("module:TeamDataProvider:ntpOffset1"); // 1-5: players
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset2");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset3");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset4");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset5");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset6");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset7");

  PLOT("module:TeamDataProvider:ntpOffset1", ntp.timeSyncBuffers[1].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset2", ntp.timeSyncBuffers[2].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset3", ntp.timeSyncBuffers[3].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset4", ntp.timeSyncBuffers[4].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset5", ntp.timeSyncBuffers[5].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset5", ntp.timeSyncBuffers[6].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset5", ntp.timeSyncBuffers[7].bestOffset);

  teamMateData = theTeamMateData;
  teamMateData.currentTimestamp = theFrameInfo.time;
  teamMateData.numOfConnectedTeamMates = 0;
  teamMateData.firstTeamMate = TeamMateData::numOfPlayers;
  int whistleHeard = 0;

  for (int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
    teamMateData.isPenalized[i] = theOwnTeamInfo.players[i - 1].penalty != PENALTY_NONE;
    teamMateData.isActive[i] = false;
    teamMateData.isFullyActive[i] = false;

    // Reset TeamMate Whistle
    // Check, if network connection is working (-> connected):
    if (teamMateData.timeStamps[i] &&
        theFrameInfo.getTimeSince(teamMateData.timeStamps[i]) < static_cast<int>(teamMateData.networkTimeout)) {
      teamMateData.numOfConnectedTeamMates++;
      // Check, if team mate is not penalized (-> active):
      if (!teamMateData.isPenalized[i]) {
        teamMateData.numOfActiveTeamMates++;
        teamMateData.isActive[i] = true;
        if (teamMateData.numOfActiveTeamMates == 1) {
          teamMateData.firstTeamMate = i;
        }
        // Check, if team mate has not been fallen down or lost ground contact (-> fully active):
        if (teamMateData.hasGroundContact[i] && teamMateData.isUpright[i]) {
          teamMateData.numOfFullyActiveTeamMates++;
          teamMateData.isFullyActive[i] = true;
        }
      }
    }

    if (theGameInfo.state != STATE_SET) {
      teamMateData.whistle[i].whistleDetected = false;
    } else if (i != theRobotInfo.number && theTeamMateData.isFullyActive[i] && theTeamMateData.whistle[i].whistleDetected) {
      whistleHeard++;
      OUTPUT_TEXT("WhistleHeard from " << i);
    }
  }

  if (whistleHeard >= 2) {
    teamMateData.whistleDetectedByTeamMates = true;
  } else {
    teamMateData.whistleDetectedByTeamMates = false;
  }
  if (teamMateData.numOfConnectedTeamMates) {
    teamMateData.wasConnected = true;
  }

  // Synchronize NTP only in ready state
  if (theGameInfo.state == STATE_READY && !ntpSynced) {
    ntpSynced = true;
    teamMateData.sendThisFrame = false;
    ntp.doSynchronization(theFrameInfo.time, Global::getTeamOut());
    lastNtpMessageSyncTime = theFrameInfo.time;
  }

  teamMateData.sendThisFrame = shouldSendThisFrame();
  if (teamMateData.sendThisFrame) {
    teamMateData.packagesToSend = toIntegral(whichPackagesToSend());
    lastSentTimeStamp = theFrameInfo.time;
  }
}

bool TeamDataProvider::shouldSendThisFrame() {
  // Ensure never to cross the budget
  if (theOwnTeamInfo.messageBudget < 15) {
    return false;
  }

  // Don't do anything when not active
  if ((theMotionRequest.motion == MotionRequest::specialAction &&
       theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead) ||
      (theMotionInfo.motionRequest.motion == MotionRequest::specialAction &&
       theMotionInfo.motionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead)) {
    return false;
  }

  // Do not send if penalized
  if (theRobotInfo.penalty != PENALTY_NONE) {
    return false;
  }

  if (theGameInfo.state == STATE_READY && theFrameInfo.time - lastNtpMessageSyncTime < 500) {
    return false;
  }

  if (thePersonalData.syncTeamRequired) {
    return true;
  }

  // Event based role change sent too many packages. Disabled for now
  // if (theBehaviorStatus.roleChanged) {
  //   return true;
  // }

  // Force send when requested
  if (shouldForceSendTimestamp != 0 && theFrameInfo.time > shouldForceSendTimestamp) {
    shouldForceSendTimestamp = 0;
    return true;
  }

  // Send when whistle is heard in set the first time
  if (theGameInfo.state == STATE_SET && theWhistle.whistleDetected && !whistleWasDetected) {
    whistleWasDetected = true;
    return true;
  }
  // Send forced when ball detected after not seeing it not for long time in playing state
  if (theGameInfo.state == STATE_PLAYING && theBallModel.timeSinceLastSeen < 100 && lastTimeSinceBallLastSeen > 5000) {
    // Give other modules a bit of time to use the new ball model to send useful data
    shouldForceSendTimestamp = theFrameInfo.time + 100.f;
  }
  lastTimeSinceBallLastSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);

  // Always send on state change if not in initial
  if (theGameInfo.state != STATE_INITIAL && theGameInfo.state != lastGameState) {
    lastGameState = theGameInfo.state;
    whistleWasDetected = false;
    // Also force another update in ready to apply synchronization, otherwise reset
    if (theGameInfo.state == STATE_READY) {
      shouldForceSendTimestamp = theFrameInfo.time + 500.f;
    } else {
      shouldForceSendTimestamp = 0;
    }
    return true;
  }

  // Otherwise do not do anything if not in playing state
  if (theGameInfo.state != STATE_PLAYING) {
    return false;
  }

  if (theFrameInfo.getTimeSince(lastSentTimeStamp) >= theTeamMateData.networkTimeout - 1000) {
    return true;
  }

  if (theBallModel.lost) {
    return false;
  }

  // Calculate packets per second to divide
  int remainingSeconds = theGameInfo.secsRemaining;
  if (theGameInfo.firstHalf) {
    remainingSeconds += 10 * 60;
  }
  remainingSeconds = std::max(remainingSeconds, 30);

  double packetsPerSecond = (thePersonalData.hasBallLock)
                              ? (hasBallLockMessageBudgetRatio * theOwnTeamInfo.messageBudget / remainingSeconds)
                              : (1 - hasBallLockMessageBudgetRatio) * theOwnTeamInfo.messageBudget / remainingSeconds / 6.0;
  return (theFrameInfo.getTimeSince(lastSentTimeStamp) >= 1000 / packetsPerSecond);
}

TeamMateDataSelector TeamDataProvider::whichPackagesToSend() {
  if (!initialDataSent && theGameInfo.state == STATE_PLAYING) {
    initialDataSent = true;
    prevRobotPose = theRobotPose;
    prevSideConfidence = theSideConfidence;
    prevBallModel = theBallModel;
    prevPlayerModel = thePlayerModel;
    prevPersonalData = thePersonalData;
    prevFallDownState = theFallDownState;
    prevGroundContactState = theGroundContactState;
    prevBehaviorStatus = theBehaviorStatus;
    return TeamMateDataSelector::all;
  }

  TeamMateDataSelector packagesToSend = TeamMateDataSelector::none;

  if ((theRobotPose.translation - prevRobotPose.translation).abs() > minPoseDistanceSendThreshold) {
    packagesToSend = packagesToSend | TeamMateDataSelector::robotPose;
    prevRobotPose = theRobotPose;
  }

  if ((std::abs(theSideConfidence.sideConfidence - prevSideConfidence.sideConfidence) > sideConfidenceValueThreshold) ||
      (theSideConfidence.mirror != prevSideConfidence.mirror) ||
      (theSideConfidence.confidenceState != prevSideConfidence.confidenceState)) {
    packagesToSend = packagesToSend | TeamMateDataSelector::sideConfidence;
    prevSideConfidence = theSideConfidence;
  }

  if (((theBallModel.lastPerception - prevBallModel.lastPerception).abs() >= minBallModelLastPerceptionSendThreshold) ||
      ((theBallModel.estimate.position - prevBallModel.estimate.position).abs() >= minBallEstimatePositionSendThreshold) ||
      (theBallModel.seenFraction >= minBallSeenFractionSendThreshold)) {
    packagesToSend = packagesToSend | TeamMateDataSelector::ballModel;
    prevBallModel = theBallModel;
  }

  if (sendPlayerModel) {
    packagesToSend = packagesToSend | TeamMateDataSelector::playerModel;
    prevPlayerModel = thePlayerModel;
  }

  if (thePersonalData.syncTeamRequired || (thePersonalData.ballLockState != prevPersonalData.ballLockState) ||
      (std::abs(thePersonalData.ballScore - prevPersonalData.ballScore) > ballScoreDifferenceSendThreshold)) {
    packagesToSend = packagesToSend | TeamMateDataSelector::personalData;
    prevPersonalData = thePersonalData;
  }

  if ((theFallDownState.state != prevFallDownState.state) ||
      (theGroundContactState.contact != prevGroundContactState.contact)) {
    packagesToSend = packagesToSend | TeamMateDataSelector::fallDownState;
    prevFallDownState = theFallDownState;
  }

  if (theBehaviorStatus.roleChanged || (theBehaviorStatus.role != prevBehaviorStatus.role) ||
      (theBehaviorStatus.lost != prevBehaviorStatus.lost)) {
    packagesToSend = packagesToSend | TeamMateDataSelector::behaviorStatus;
    prevBehaviorStatus = theBehaviorStatus;
  }

  return packagesToSend;
}

void TeamDataProvider::handleMessages(MessageQueue& teamReceiver) {
  if (theInstance) {
    teamReceiver.handleAllMessages(*theInstance);
  }

  teamReceiver.clear();
}

bool TeamDataProvider::handleMessage(InMessage& message) {
  /*
  The robotNumber and the three flags hasGroundContact, isUpright and isPenalized should always be updated.
   */
  switch (message.getMessageID()) {

  case idNTPHeader:
    VERIFY(ntp.handleMessage(message));
    timeStamp = ntp.receiveTimeStamp;
    return false;
  case idNTPIdentifier:
  case idNTPRequest:
  case idNTPResponse:
    return ntp.handleMessage(message);

  case idRobot:
    message.bin >> robotNumber;
    if (robotNumber != theRobotInfo.number) {
      if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
        theTeamMateData.timeStamps[robotNumber] = timeStamp;
      }
    }
    return true;

  case idGroundTruthWorldState: {
    message.bin >> theGroundTruthWorldState;
    return true;
  }

  case idTeamMateHasGroundContact:
    if (robotNumber != theRobotInfo.number) {
      if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
        char hasGroundContact;
        message.bin >> hasGroundContact;
        theTeamMateData.hasGroundContact[robotNumber] = hasGroundContact;
        // This is a potentially evil quick workaround that should be replaced by a better handling of ground contacts of
        // team mates
        // at many different places in our code! For a detailed problem description, ask Tim.
        if (!theTeamMateData.hasGroundContact[robotNumber]) {
          theTeamMateData.hasGroundContact[robotNumber] =
            theFrameInfo.getTimeSince(theTeamMateData.timeLastGroundContact[robotNumber]) < 2000;
        }
      }
    }
    return true;

  case idTeamMateIsUpright:
    if (robotNumber != theRobotInfo.number) {
      if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
        char isUpright;
        message.bin >> isUpright;
        theTeamMateData.isUpright[robotNumber] = isUpright;
      }
    }
    return true;

  case idTeamMateTimeSinceLastGroundContact:
    if (robotNumber != theRobotInfo.number) {
      if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
        message.bin >> theTeamMateData.timeLastGroundContact[robotNumber];
        REMOTE_TO_LOCAL_TIME(theTeamMateData.timeLastGroundContact[robotNumber]);
      }
    }
    return true;

  case idPersonalData:
    if (robotNumber != theRobotInfo.number) {
      if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
        UNPACK(PersonalData, robotsPersonalData);
        // message.bin >> theTeamMateData.robotsPersonalData[robotNumber];
      }
    }
    return true;

  case idWhistle:
    if (robotNumber != theRobotInfo.number) {
      if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
        message.bin >> theTeamMateData.whistle[robotNumber];
        REMOTE_TO_LOCAL_TIME(theTeamMateData.whistle[robotNumber].lastTimeWhistleDetected);
        REMOTE_TO_LOCAL_TIME(theTeamMateData.whistle[robotNumber].lastTimeOfIncomingSound);
      }
    }
    return true;
  }

  /*
  The messages in the following switch block should only be updated
  if hasGroundContact == true and isPenalized == false, because the information of this message
  can only be reliable if the robot is actively playing.
   */
  if (!theTeamMateData.isPenalized[robotNumber] && theTeamMateData.hasGroundContact[robotNumber]) {
    switch (message.getMessageID()) {
    case idTeamMateBallModel:
      if (robotNumber != theRobotInfo.number) {
        if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
          UNPACK(BallModel, ballModels);
          BallModel& ballModel = theTeamMateData.ballModels[robotNumber];
          REMOTE_TO_LOCAL_TIME(ballModel.timeWhenLastSeen);
          REMOTE_TO_LOCAL_TIME(ballModel.timeWhenDisappeared);
        }
      }
      return true;

    case idTeamMatePlayerModel:
      if (robotNumber != theRobotInfo.number) {
        if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
          UNPACK(PlayerModel, playerModels);
          for (size_t i = 0; i < theTeamMateData.playerModels[robotNumber].players.size(); i++) {
            REMOTE_TO_LOCAL_TIME(theTeamMateData.playerModels[robotNumber].players[i].timeStamp);
          }
        }
      }
      return true;

    case idTeamMateRobotPose:
      if (robotNumber != theRobotInfo.number) {
        if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
          UNPACK(RobotPose, robotPoses);
        }
      }
      return true;

    case idTeamMateSideConfidence:
      if (robotNumber != theRobotInfo.number) {
        if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
          message.bin >> theTeamMateData.robotsSideConfidence[robotNumber];
        }
      }
      return true;

    case idTeamMateBehaviorStatus:
      if (robotNumber != theRobotInfo.number) {
        if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
          message.bin >> theTeamMateData.behaviorStatus[robotNumber];
        }
      }
      return true;

    case idTeamHeadControl:
      if (robotNumber != theRobotInfo.number) {
        if (robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers) {
          message.bin >> theTeamMateData.teamHeadControlStates[robotNumber];
        }
      }
      return true;
    }
  }
  return true;
}

MAKE_MODULE(TeamDataProvider, Cognition Infrastructure)
