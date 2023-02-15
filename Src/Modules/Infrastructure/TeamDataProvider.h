/**
 * @file TeamDataProvider.h
 *
 * This file implements a module that provides the data received by team communication.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Core/MessageQueue/MessageQueue.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/GroundTruthWorldState.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/NTP.h"

MODULE(TeamDataProvider)
REQUIRES(FrameInfo)
REQUIRES(RobotInfo)
REQUIRES(GameInfo)
REQUIRES(OwnTeamInfo)
REQUIRES(MotionInfo)
REQUIRES(BallModel)
USES(MotionRequest)
PROVIDES_WITH_MODIFY_AND_OUTPUT(GroundTruthWorldState)
PROVIDES_WITH_MODIFY_AND_DRAW(TeamMateData)
END_MODULE

class TeamDataProvider : public TeamDataProviderBase, public MessageHandler {
private:
  static PROCESS_WIDE_STORAGE(TeamDataProvider)
    theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  unsigned timeStamp;                    /**< The time when the messages currently processed were received. */
  int robotNumber;                       /**< The number of the robot the messages of which are currently processed. */
  unsigned lastSentTimeStamp;            /**< The time when the last package to teammates was sent. */
  int lastGameState{-1};                 /**< Last game state when checking to send. */
  unsigned lastTimeSinceBallLastSeen{0}; /** < The time the robot saw the ball the last time. */
  unsigned shouldForceSendTimestamp{0};  /** < The time a forced message should be send. */

  NTP ntp;                                        /**< The Network Time Protocol. */
  GroundTruthWorldState theGroundTruthWorldState; /**< The last ground truth world state received. */
  TeamMateData theTeamMateData;                   /**< The last received team mate data (ball model, robot pose, etc.). */

  void update(GroundTruthWorldState& groundTruthWorldState) { groundTruthWorldState = theGroundTruthWorldState; }
  void update(TeamMateData& teamMateData);

  /**
   * The method is called for every incoming team message by handleMessages.
   * @param message An interface to read the message from the queue.
   * @return true Was the message handled?
   */
  bool handleMessage(InMessage& message);

  /**
   * Determines if message should be send in this frame.
   */
  bool shouldSendThisFrame();

public:
  /**
   * Default constructor.
   */
  TeamDataProvider();

  /**
   * Destructor.
   */
  ~TeamDataProvider();

  /**
   * The method is called to handle all incoming team messages.
   * @param teamReceiver The message queue containing all team messages received.
   */
  static void handleMessages(MessageQueue& teamReceiver);
};
