/**
 * @file CombinedWorldModelProvider.cpp
 *
 * Implementation of the CombinedWorldModelProvider module.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Katharina Gillmann
 */

#include "CombinedWorldModelProvider.h"
#include "Core/Debugging/DebugDrawings.h"
#include <algorithm>
#include <climits>
#include "Core/Streams/InStreams.h"
#include "Core/Math/GaussianDistribution.h"
#include "Core/Math/Matrix2x2.h"
#include "Core/Math/Vector2.h"
#include <string.h>
#include <iostream>
#include <fstream>
#include "Representations/Configuration/FieldDimensions.h"

using namespace std;

#define UPDATE_STEP_MS 1000

MAKE_MODULE(CombinedWorldModelProvider, Modeling)

void CombinedWorldModelProvider::update(CombinedWorldModel& combinedWorldModel) {
  DECLARE_DEBUG_DRAWING("module:CombinedWorldModelProvider", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:CombinedWorldModelProvider:ProvidedInfo", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:CombinedWorldModelProvider:Opponents", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:CombinedWorldModelProvider:Allies", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:CombinedWorldModelProvider:Ball", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:CombinedWorldModelProvider:BallOthers", "drawingOnField");

  // initialize with previous result
  combinedWorldModel = theCombinedWorldModel;

  const int me = theRobotInfo.number;
  unsigned timeStep = theFrameInfo.getTimeSince(latestUpdate); // time since the last update

  /*------------------------------Provide a list with the available allies---------------------------*/
  // sets player availability (canTrust, canPlay)
  // updates the position for trusted allies
  // checks whether new and reliable data has arrived
  std::vector<unsigned char>& canTrust = combinedWorldModel.canTrust;
  std::vector<unsigned char>& canPlay = combinedWorldModel.canPlay;
  canTrust = std::vector<unsigned char>(TeamMateData::numOfPlayers);
  canPlay = std::vector<unsigned char>(TeamMateData::numOfPlayers);

  for (unsigned i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
    if (i == me) {
      allies[i].update(theRobotPose, i);
      arrivalTimes[i] = theFrameInfo.time;
      // the "penalized" state is checked twice (by checking the message of the team mate itself and by checking the state
      // set by the GameController). The index of theOwnTeamInfo.players is i-1 because it is "zero based" whereas
      // theTeamMateData is "one based"
      if (theRobotInfo.penalty == PENALTY_NONE && theOwnTeamInfo.players[i - 1].penalty == PENALTY_NONE) {
        canPlay[i] = true;
        // What does the validity value actually provide??
        // Additionally take side confidence into consideration???
        if (theFallDownState.state == FallDownState::upright && theRobotPose.validity > validityThreshold) {
          canTrust[i] = true;
        }
      }
    } else {
      canPlay[i] = theTeamMateData.isActive[i];
      allies[i].update(theTeamMateData.robotPoses[i], i);
      arrivalTimes[i] = theTeamMateData.timeStamps[i];
      canTrust[i] = theTeamMateData.isFullyActive[i] && theTeamMateData.robotPoses[i].validity > validityThreshold;
    }
  }

  if (latestUpdate == 0 || timeStep > UPDATE_STEP_MS) {
    /*------------------------update movement/ position prediction of the tracked opponents---------*/
    for (auto& robot : potentialOpponents) {
      robot.predictMovement(timeStep);
    }

    /*--------------------------------combine the measurements of Robots----------------------------*/
    // collect all robots, that have been detected by the own team
    for (unsigned i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
      // The detected robots are only used when the ally is fully active and when its new information
      if (!canTrust[i]) {
        continue;
      }

      // get the detections
      std::vector<PlayerModel::Player> detectedPlayers;

      if (i == me) {
        detectedPlayers = thePlayerModel.players;
      } else {
        detectedPlayers = theTeamMateData.playerModels[i].players;
      }

      Ally observer = allies[i];
      for (auto& player : detectedPlayers) {
        // ignore detections in the future (if time is not synchronized)
        if (player.timeStamp > theFrameInfo.time) {
          continue;
        }
        // only use recent detections
        if (theFrameInfo.getTimeSince(player.timeStamp) > theTeamMateData.networkTimeout) {
          continue;
        }

        Detection detect =
          Detection(observer.state, observer.covar, player.relPosOnField, player.covariance, player.timeStamp);
        detect.distance = 20000; // start with a distance of 20 m
        detect.assignedTo = Assignment::NONE;
        // assign measurement to teammates
        for (int j = TeamMateData::firstPlayer; j < TeamMateData::numOfPlayers; j++) {
          if (i != j && canPlay[j]) {
            auto allyDis = allies[j].distance(detect);
            if (allyDis < detect.distance && allyDis < playerDistanceThreshold) {
              detect.assignmentType = Assignment::ALLY;
              detect.assignedTo = j;
              detect.distance = allyDis;
            }
          }
        }
        // assign the measurement to tracked robots
        for (auto it = potentialOpponents.begin(); it != potentialOpponents.end(); it++) {
          auto oppDis = it->distance(detect);
          if (oppDis < detect.distance && oppDis < playerDistanceThreshold) {
            detect.assignedTo = std::distance(potentialOpponents.begin(), it);
            detect.assignmentType = Assignment::OPPONENT;
            detect.distance = oppDis;
          }
        }

        // handle the unassigned detections
        switch (detect.assignmentType) {
        case Assignment::NONE:
          if (detect.distance > initDistance) {
            potentialOpponents.push_back(TrackedRobot(detect));
            potentialOpponents.back().lastSeen[i] = detect.timeStamp;
          }
          break;
        case Assignment::ALLY:
          if (detect.timeStamp <= allies[detect.assignedTo].lastSeen[i]) {
            break;
          }
          allies[detect.assignedTo].merge(detect);
          allies[detect.assignedTo].hasBeenDetected = true;
          allies[detect.assignedTo].lastSeen[i] = detect.timeStamp;
          break;
        case Assignment::OPPONENT:
          if (detect.timeStamp <= potentialOpponents[detect.assignedTo].lastSeen[i]) {
            break;
          }
          potentialOpponents[detect.assignedTo].merge(detect);
          potentialOpponents[detect.assignedTo].lastSeen[i] = detect.timeStamp;
          break;
        default:
          break;
        }
      }
    }

    // stop and start tracking of Robots
    auto it = potentialOpponents.begin();
    opponents.clear();
    while (it != potentialOpponents.end()) {
      if (theFrameInfo.getTimeSince(it->latestUpdate) > stopTrackTime || isPlayerOutsideOfField(*it) ||
          it->getTotalVar() > maxTotalVar) {
        it = potentialOpponents.erase(it);
      } else if (theFrameInfo.getTimeSince(it->firstSeen) > trackCreationTime) {
        opponents.push_back(*it);
        it++;
      } else {
        it++;
      }
    }

    // fill in the Robot position information of the CombinedWorldModel Representation
    combinedWorldModel.positionsOwnTeam.clear(); // this contains only the positions of the fully active alies
    combinedWorldModel.ownPoses.resize(TeamMateData::numOfPlayers);
    for (int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
      combinedWorldModel.ownPoses[i] = allies[i].state;
      if (canTrust[i]) {
        combinedWorldModel.positionsOwnTeam.push_back(allies[i].state);
      }
    }

    // fill in the Positions of the opponents
    combinedWorldModel.positionsOpponentTeam.clear();
    for (auto& robot : opponents) {
      combinedWorldModel.positionsOpponentTeam.push_back(robot.getGPD());
    }

    /*------------------------- Combining the Ball Information --------------------------------------*/
    // reset the seen fraction and the active detections
    for (auto& potBallModel : potentialBallModels) {
      for (unsigned i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
        potBallModel.seenFractions[i] = 0.0f;
      }
      potBallModel.seenFraction = 0.0f;
      potBallModel.numberOfActiveDetections = 0;
    }

    // collect all ball positions
    for (unsigned i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
      // The detected balls are only used when the ally is fully active and when its new information
      if (!canTrust[i]) {
        continue;
      }

      BallModel ballModel;
      float sideConfidence = 0;
      if (i == me) {
        ballModel = theBallModel;
      } else {
        ballModel = theTeamMateData.ballModels[i];
        sideConfidence = theTeamMateData.robotsSideConfidence[i].sideConfidence;
      }

      // check detections in the future
      if (ballModel.timeWhenLastSeen > theFrameInfo.time) {
        // detection is only little in the future, this might happen due to instable time synchronization
        if (ballModel.timeWhenLastSeen - theFrameInfo.time <= theTeamMateData.networkTimeout) {
          // approximate the detection as half way in previous frame...
          ballModel.timeWhenLastSeen = theFrameInfo.time - theTeamMateData.networkTimeout / 2;
        } else {
          // the time is not synchronized
          continue;
        }
      }
      // only use recent ball updates
      if (theFrameInfo.getTimeSince(ballModel.timeWhenLastSeen) > theTeamMateData.networkTimeout) {
        continue;
      }
      // only use ball updates with a certain minimum fraction
      if (ballModel.seenFraction < minSeenFraction) {
        continue;
      }

      // convert to global coordinates (FIXME: implement this at proper place)
      // use the last observed position instead of the updated one
      Ally observer = allies[i];
#ifdef TARGET_ROBOT
      ballModel.estimate.position = allies[i].state * ballModel.lastPerception;
#else
      // NOTE: last perception is not defined properly in simulation
      ballModel.estimate.position = allies[i].state * ballModel.estimate.position;
#endif
      ballModel.estimate.velocity = {0, 0}; // FIXME: not used

      // bool assigned = false;
      int closestBall = -1;
      float minBallDist = INFINITY;
      bool assigned = false;
      for (unsigned j = 0; j < potentialBallModels.size(); ++j) {
        auto& potBallModel = potentialBallModels[j];
        int ballDist = potBallModel.dist(ballModel);
        if (ballDist < ballDistanceThreshold) {
          if (potBallModel.lastSeen[i] < ballModel.timeWhenLastSeen) {
            if (ballDist < minBallDist) {
              minBallDist = ballDist;
              closestBall = j;
            }
            potBallModel.lastSeen[i] = ballModel.timeWhenLastSeen;
          }
          assigned = true;
        }
      }
      if (!assigned) {
        potentialBallModels.push_back(ballModel);
        auto potBallModel = potentialBallModels.back();
        potBallModel.seenByMe = (i == me);
        potBallModel.lastSeen[i] = ballModel.timeWhenLastSeen;
        potBallModel.seenFractions[i] = ballModel.seenFraction;
        if (i != me) {
          potBallModel.maxSideConfidenceOthers = sideConfidence;
        }
      } else if (closestBall != -1) {
        potentialBallModels[closestBall].merge(ballModel, i == me, sideConfidence);
        potentialBallModels[closestBall].seenFractions[i] = ballModel.seenFraction;
      }
    }

    // stop tracking of ball positions and select the best estimate
    int selectedBall = -1;
    int selectedBallOthers = -1;
    auto ballIt = potentialBallModels.begin();
    while (ballIt != potentialBallModels.end()) {
      // update the total seen fraction in the last frame
      ballIt->updateSeenFraction();

      // calculate number of players that saw it before delete time
      bool seenByOther = false;
      ballIt->numberOfDetections = 0;
      for (unsigned i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
        if (theFrameInfo.getTimeSince(ballIt->lastSeen[i]) <= deleteBallTime) {
          ballIt->numberOfDetections++;
          if (i != me) {
            seenByOther = true;
          }
        }
      }

      if (theFrameInfo.getTimeSince(ballIt->timeWhenLastSeen) > deleteBallTime) {
        ballIt = potentialBallModels.erase(ballIt);
        continue;
      }

      if (selectedBall == -1 || ballIt->isBetter(potentialBallModels[selectedBall], false)) {
        selectedBall = ballIt - potentialBallModels.begin();
      }
      if (seenByOther && (selectedBallOthers == -1 || ballIt->isBetter(potentialBallModels[selectedBallOthers], true))) {
        selectedBallOthers = ballIt - potentialBallModels.begin();
      }

      ballIt++;
    }

    if (selectedBall != -1) { // ball has been seen by at least one ally or itself
      combinedWorldModel.ballIsValid = true;
      selectedBallModel = potentialBallModels[selectedBall];

      combinedWorldModel.ballState = potentialBallModels[selectedBall].estimate;
      combinedWorldModel.expectedEndPosition = combinedWorldModel.ballState.getEndPosition(theFieldDimensions.ballFriction);
    } else {
      combinedWorldModel.ballIsValid = false;
    }

    if (selectedBallOthers != -1) { // at least one ball seen by ally (ignoring ourselves)
      combinedWorldModel.ballIsValidOthers = true;
      selectedBallModelOthers = potentialBallModels[selectedBallOthers];
      combinedWorldModel.ballStateOthersMaxSideConfidence = potentialBallModels[selectedBallOthers].maxSideConfidenceOthers;
      combinedWorldModel.ballStateOthers = potentialBallModels[selectedBallOthers].estimate;
    } else {
      combinedWorldModel.ballIsValidOthers = false;
    }

    // -----------------------set latest update---------------------------------------------------
    latestUpdate = theFrameInfo.time;
  }

  // update ball last seen
  if (selectedBallModel.timeWhenLastSeen) {
    combinedWorldModel.timeSinceBallLastSeen = theFrameInfo.getTimeSince(selectedBallModel.timeWhenLastSeen);
    combinedWorldModel.timeSinceBallLastSeenPlayer.clear();
    for (unsigned i = 0; i < TeamMateData::numOfPlayers; ++i) {
      combinedWorldModel.timeSinceBallLastSeenPlayer.push_back(theFrameInfo.getTimeSince(selectedBallModel.lastSeen[i]));
    }
  } else {
    combinedWorldModel.timeSinceBallLastSeen = INT_MAX;
    combinedWorldModel.timeSinceBallLastSeenPlayer.clear();
    for (unsigned i = 0; i < TeamMateData::numOfPlayers; ++i) {
      combinedWorldModel.timeSinceBallLastSeenPlayer.push_back(INT_MAX);
    }
  }
  if (selectedBallModelOthers.timeWhenLastSeen) {
    combinedWorldModel.timeSinceBallLastSeenOthers = theFrameInfo.getTimeSince(selectedBallModelOthers.timeWhenLastSeen);
    combinedWorldModel.timeSinceBallOthersLastSeenPlayer.clear();
    for (unsigned i = 0; i < TeamMateData::numOfPlayers; ++i) {
      combinedWorldModel.timeSinceBallOthersLastSeenPlayer.push_back(
        theFrameInfo.getTimeSince(selectedBallModelOthers.lastSeen[i]));
    }
  } else {
    combinedWorldModel.timeSinceBallOthersLastSeenPlayer.clear();
    for (unsigned i = 0; i < TeamMateData::numOfPlayers; ++i) {
      combinedWorldModel.timeSinceBallOthersLastSeenPlayer.push_back(INT_MAX);
    }
  }

  /*-------------------------Drawings-------------------------------------------------------------*/
  // available information
  for (auto p : thePlayerModel.players) {
    Vector2<> globalPos = theRobotPose * p.relPosOnField;
    CIRCLE("module:CombinedWorldModelProvider:ProvidedInfo",
           globalPos.x,
           globalPos.y,
           150,
           20,
           Drawings::ps_solid,
           ColorClasses::blue,
           Drawings::bs_null,
           ColorRGBA());
    LINE("module:CombinedWorldModelProvider:ProvidedInfo",
         theRobotPose.translation.x,
         theRobotPose.translation.y,
         globalPos.x,
         globalPos.y,
         25,
         Drawings::ps_dash,
         ColorClasses::blue);
  }
  Vector2<> globalBall = theRobotPose * theBallModel.estimate.position;
  CIRCLE("module:CombinedWorldModelProvider:ProvidedInfo",
         globalBall.x,
         globalBall.y,
         100,
         20,
         Drawings::ps_solid,
         ColorClasses::yellow,
         Drawings::bs_null,
         ColorRGBA());
  LINE("module:CombinedWorldModelProvider:ProvidedInfo",
       theRobotPose.translation.x,
       theRobotPose.translation.y,
       globalBall.x,
       globalBall.y,
       25,
       Drawings::ps_dash,
       ColorClasses::yellow);

  for (int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; i++) {
    if (canPlay[i] && i != me) {
      CIRCLE("module:CombinedWorldModelProvider:ProvidedInfo",
             theTeamMateData.robotPoses[i].translation.x,
             theTeamMateData.robotPoses[i].translation.y,
             150,
             20,
             Drawings::ps_solid,
             ColorClasses::green,
             Drawings::bs_null,
             ColorRGBA());

      for (int k = 0; k < (int)theTeamMateData.playerModels[i].players.size(); k++) {
        Vector2<> globalPos = theTeamMateData.robotPoses[i] * theTeamMateData.playerModels[i].players[k].relPosOnField;

        CIRCLE("module:CombinedWorldModelProvider:ProvidedInfo",
               globalPos.x,
               globalPos.y,
               150,
               20,
               Drawings::ps_solid,
               ColorClasses::red,
               Drawings::bs_null,
               ColorRGBA());
        LINE("module:CombinedWorldModelProvider:ProvidedInfo",
             theTeamMateData.robotPoses[i].translation.x,
             theTeamMateData.robotPoses[i].translation.y,
             globalPos.x,
             globalPos.y,
             25,
             Drawings::ps_dash,
             ColorClasses::red);
      }

      Vector2<> globalBall = theTeamMateData.robotPoses[i] * theTeamMateData.ballModels[i].lastPerception;
      CIRCLE("module:CombinedWorldModelProvider:ProvidedInfo",
             globalBall.x,
             globalBall.y,
             75,
             20,
             Drawings::ps_solid,
             ColorClasses::orange,
             Drawings::bs_null,
             ColorRGBA());
      LINE("module:CombinedWorldModelProvider:ProvidedInfo",
           theTeamMateData.robotPoses[i].translation.x,
           theTeamMateData.robotPoses[i].translation.y,
           globalBall.x,
           globalBall.y,
           25,
           Drawings::ps_dash,
           ColorClasses::orange);
    }
  }
  // calculated results
  for (int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; i++) {
    if (canPlay[i]) {
      auto pos = combinedWorldModel.ownPoses[i].translation;
      SIMPLE_COVARIANCE2D("module:CombinedWorldModel", allies[i].covar, pos, 10, Drawings::ps_solid, ColorClasses::orange);
      CIRCLE("module:CombinedWorldModelProvider:Allies",
             pos.x,
             pos.y,
             150,
             20,
             Drawings::ps_solid,
             ColorClasses::blue,
             Drawings::bs_null,
             ColorRGBA());
      for (int j = TeamMateData::firstPlayer; j < TeamMateData::numOfPlayers; j++) {
        if (theFrameInfo.getTimeSince(allies[i].lastSeen[j]) < theTeamMateData.networkTimeout) {
          auto posDetector = combinedWorldModel.ownPoses[j].translation;
          auto color = (j == me) ? ColorClasses::blue : ColorClasses::red;
          LINE("module:CombinedWorldModelProvider:Allies",
               pos.x,
               pos.y,
               posDetector.x,
               posDetector.y,
               25,
               Drawings::ps_dash,
               color);
        }
      }
    }
  }
  for (auto it = opponents.begin(); it != opponents.end(); it++) {
    Vector2f globalPos = it->getPosition();
    SIMPLE_COVARIANCE2D(
      "module:CombinedWorldModel", it->getCovar(), globalPos, 10, Drawings::ps_solid, ColorClasses::orange);
    CIRCLE("module:CombinedWorldModelProvider:Opponents",
           globalPos.x,
           globalPos.y,
           150,
           20,
           Drawings::ps_solid,
           ColorClasses::yellow,
           Drawings::bs_null,
           ColorRGBA());
    for (int j = TeamMateData::firstPlayer; j < TeamMateData::numOfPlayers; j++) {
      if (theFrameInfo.getTimeSince(it->lastSeen[j]) < theTeamMateData.networkTimeout) {
        auto posDetector = combinedWorldModel.ownPoses[j].translation;
        auto color = (j == me) ? ColorClasses::blue : ColorClasses::red;
        LINE("module:CombinedWorldModelProvider:Opponents",
             globalPos.x,
             globalPos.y,
             posDetector.x,
             posDetector.y,
             25,
             Drawings::ps_dash,
             color);
      }
    }
  }
  if (combinedWorldModel.ballIsValid) {
    CIRCLE("module:CombinedWorldModelProvider:Ball",
           selectedBallModel.estimate.position.x,
           selectedBallModel.estimate.position.y,
           100,
           20,
           Drawings::ps_solid,
           ColorClasses::orange,
           Drawings::bs_null,
           ColorRGBA());
    for (int j = TeamMateData::firstPlayer; j < TeamMateData::numOfPlayers; j++) {
      if (theFrameInfo.getTimeSince(selectedBallModel.lastSeen[j]) < theTeamMateData.networkTimeout) {
        auto posDetector = combinedWorldModel.ownPoses[j].translation;
        auto color = (j == me) ? ColorClasses::blue : ColorClasses::red;
        LINE("module:CombinedWorldModelProvider:Ball",
             selectedBallModel.estimate.position.x,
             selectedBallModel.estimate.position.y,
             posDetector.x,
             posDetector.y,
             25,
             Drawings::ps_dash,
             color);
      }
    }
  }
  if (combinedWorldModel.ballIsValidOthers) {
    CIRCLE("module:CombinedWorldModelProvider:BallOthers",
           selectedBallModelOthers.estimate.position.x,
           selectedBallModelOthers.estimate.position.y,
           50,
           20,
           Drawings::ps_solid,
           ColorClasses::red,
           Drawings::bs_null,
           ColorRGBA());
    for (int j = TeamMateData::firstPlayer; j < TeamMateData::numOfPlayers; j++) {
      if (theFrameInfo.getTimeSince(selectedBallModel.lastSeen[j]) < theTeamMateData.networkTimeout) {
        auto posDetector = combinedWorldModel.ownPoses[j].translation;
        auto color = (j == me) ? ColorClasses::blue : ColorClasses::red;
        LINE("module:CombinedWorldModelProvider:BallOthers",
             selectedBallModelOthers.estimate.position.x,
             selectedBallModelOthers.estimate.position.y,
             posDetector.x,
             posDetector.y,
             25,
             Drawings::ps_dash,
             color);
      }
    }
  }

  //-------------write log file-----------------------------
  if (enableLogging) {
    std::ofstream myfile;
    myfile.open("/home/nao/logs/SelfLoc.log", std::ios::app);
    myfile << theFrameInfo.time;
    myfile << ", " << theRobotPose.validity << "," << theRobotPose.covariance[0][0] << ", " << theRobotPose.covariance[1][0]
           << ", " << theRobotPose.covariance[0][1] << ", " << theRobotPose.covariance[1][1];
    myfile << std::endl;
    myfile.close();

    if (latestUpdate == theFrameInfo.time) {
      std::ofstream myfile;
      myfile.open("/home/nao/logs/CombinedWorldModel.log", std::ios::app);
      myfile << theFrameInfo.time << ", " << opponents.size() << ", " << potentialOpponents.size();
      for (int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
        if (i == me) {
          myfile << ", " << theRobotPose.validity << "," << theRobotPose.translation.x << ", "
                 << theRobotPose.translation.y; // selfLoc
        } else {
          myfile << "," << theTeamMateData.robotPoses[i].validity << "," << theTeamMateData.robotPoses[i].translation.x
                 << ", " << theTeamMateData.robotPoses[i].translation.y; // selfLoc
        }
        myfile << "," << allies[i].hasBeenDetected << ", " << allies[i].state.translation.x << ", "
               << allies[i].state.translation.y; // updated position
      }
      for (auto& robot : opponents) {
        myfile << ", t, " << robot.getPosition().x << ", " << robot.getPosition().y;
      }
      myfile << std::endl;
      myfile.close();
    }
  }
}

bool CombinedWorldModelProvider::isPlayerOutsideOfField(TrackedRobot& robot) {
  Vector2f globalPlayerPos = robot.getPosition();

  // xAxis check
  if (abs(globalPlayerPos.x) > theFieldDimensions.xPosOpponentGroundline + playerOutsideFieldOffset) {
    return true;
  }

  // yAxis check
  if (abs(globalPlayerPos.y) > theFieldDimensions.yPosLeftSideline + playerOutsideFieldOffset) {
    return true;
  }

  return false;
}
