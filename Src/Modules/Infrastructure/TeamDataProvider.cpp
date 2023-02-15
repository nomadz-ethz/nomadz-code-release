/**
 * @file TeamDataProvider.cpp
 *
 * This file implements a module that provides the data received by team communication.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
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

TeamDataProvider::TeamDataProvider() : timeStamp(0), robotNumber(-1), lastSentTimeStamp(0) {
  theInstance = this;
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

  PLOT("module:TeamDataProvider:ntpOffset1", ntp.timeSyncBuffers[1].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset2", ntp.timeSyncBuffers[2].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset3", ntp.timeSyncBuffers[3].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset4", ntp.timeSyncBuffers[4].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset5", ntp.timeSyncBuffers[5].bestOffset);

  teamMateData = theTeamMateData;
  teamMateData.currentTimestamp = theFrameInfo.time;
  teamMateData.numOfConnectedTeamMates = 0;
  teamMateData.firstTeamMate = TeamMateData::numOfPlayers;
  for (int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
    teamMateData.isPenalized[i] = theOwnTeamInfo.players[i - 1].penalty != PENALTY_NONE;
    teamMateData.isActive[i] = false;
    teamMateData.isFullyActive[i] = false;
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
  }
  if (teamMateData.numOfConnectedTeamMates) {
    teamMateData.wasConnected = true;
  }

  teamMateData.sendThisFrame = shouldSendThisFrame();

  if (teamMateData.sendThisFrame) {
    ntp.doSynchronization(theFrameInfo.time, Global::getTeamOut());
    lastSentTimeStamp = theFrameInfo.time;
    shouldForceSendTimestamp = 0;
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

  // Force send when requested
  if (shouldForceSendTimestamp != 0 && theFrameInfo.time > shouldForceSendTimestamp) {
    return true;
  }

  // Send forced when ball detected after not seeing it not for long time
  if (theBallModel.timeSinceLastSeen < 100 && lastTimeSinceBallLastSeen > 5000) {
    // Give other modules a bit of time to use the new ball model to send useful data
    shouldForceSendTimestamp = theFrameInfo.time + 100.f;
  }
  lastTimeSinceBallLastSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);

  // Always send on state change if not in initial unless sent recently
  if (theGameInfo.state != STATE_INITIAL && theGameInfo.state != lastGameState &&
      theFrameInfo.getTimeSince(lastSentTimeStamp) > 2500.f) {
    lastGameState = theGameInfo.state;
    return true;
  }

  // Otherwise do not do anything if not in playing state
  if (theGameInfo.state != STATE_PLAYING) {
    return false;
  }

  // Calculate packets per second to divide
  int remainingSeconds = theGameInfo.secsRemaining;
  if (theGameInfo.firstHalf) {
    remainingSeconds += 10 * 60;
  }
  remainingSeconds = std::max(remainingSeconds, 30);

  double packetsPerSecond = theOwnTeamInfo.messageBudget / 5.0 / remainingSeconds;
  return (theFrameInfo.getTimeSince(lastSentTimeStamp) >= 1000 / packetsPerSecond);
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
        message.bin >> theTeamMateData.hasGroundContact[robotNumber];
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
        message.bin >> theTeamMateData.isUpright[robotNumber];
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
        message.bin >> theTeamMateData.robotsPersonalData[robotNumber];
        REMOTE_TO_LOCAL_TIME(theTeamMateData.robotsPersonalData[robotNumber].lastKickTime);
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
