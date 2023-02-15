/**
 * @file GameInfoProvider.cpp
 *
 * Provides the GameController packages, but makes sure the "set" or "play" game state is accurate.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include "GameInfoProvider.h"
GameInfoProvider::GameInfoProvider() : timeWhistleHeard(-1), whistleHeard(false), ballInPlay(true) {}
void GameInfoProvider::update(GameInfo& gameInfo) {

  gameInfo = theRawGameInfo;

  if (theRawGameInfo.state == STATE_READY) {
    whistleHeard = false;
  } else if (theRawGameInfo.state == STATE_SET && theRawGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT &&
             theWhistle.whistleDetected == true && whistleHeard == false) {
    gameInfo.state = STATE_PLAYING;
    whistleHeard = true;
    timeWhistleHeard = theFrameInfo.time;
    if (theOwnTeamInfo.teamNumber != theRawGameInfo.kickingTeam) {
      // Other team has kickoff
      ballInPlay = false;
    } else {
      // We have kickoff
      ballInPlay = true;
    }
  }

  if (theRawGameInfo.state == STATE_SET && whistleHeard == true) {
    gameInfo.state = STATE_PLAYING;
  }

  const Vector2<> gloBall = theRobotPose * theBallModel.estimate.position;
  if (gameInfo.state == STATE_PLAYING && ballInPlay == false) { // THIS IS NOT ACTUAL BALL POSITION - ONLY RELATIVE TO ROBOT

    // Ball seen, and is more than a certain distance from origin
    // if (theBallModel.timeSinceLastSeen < 3000 && gloBall.abs() > ballTouchedThreshold) {
    //   ballInPlay = true;
    //   SystemCall::playSound("ballIsFree.wav");
    // }

    // More than 10s since whistle heard
    if (theFrameInfo.time - timeWhistleHeard > 10000) {
      ballInPlay = true; //|| ((theFrameInfo.time - theBallModel.timeWhenLastSeen < 100)  && (theRobotPose *
                         // theBallModel.estimate.position).abs() > 350))
      SystemCall::playSound("ballIsFree.wav");
    }
  }
  gameInfo.ballInPlay = ballInPlay;

  // std::cout << "raw state: " << (int)theRawGameInfo.state << ", translated state: " << (int)gameInfo.state << std::endl;
  // std::cout << "whistleHeard: " << whistleHeard << ", time since whistle heard: " << (theFrameInfo.time -
  // timeWhistleHeard) << std::endl;
  // std::cout << "gloBall.abs(): " << gloBall.abs() << std::endl;
  // std::cout << "ballInPlay: " << ballInPlay << std::endl;
}

MAKE_MODULE(GameInfoProvider, Cognition Infrastructure)
