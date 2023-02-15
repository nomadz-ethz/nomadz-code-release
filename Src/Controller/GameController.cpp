/**
 * @file GameController.h
 *
 * This file implements a class that simulates a console-based GameController.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "GameController.h"
#include "Oracle.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"
#include "Representations/Modeling/RobotPose.h"
#include "Core/Streams/InStreams.h"
#include "Core/System/BHAssert.h"
#include <limits>
#include <algorithm>

FieldDimensions GameController::fieldDimensions;

// move to header when MSC supports constexpr
const float GameController::footLength = 120.f;
const float GameController::safeDistance = 150.f;
const float GameController::dropHeight = 350.f;

GameController::GameController() : timeOfLastDropIn(0), timeWhenLastRobotMoved(0), timeWhenStateBegan(0), automatic(true) {
  gameInfo.firstHalf = 1;
  gameInfo.kickingTeam = 1;
  gameInfo.secsRemaining = durationOfHalf;
  teamInfos[homeTeam].teamNumber = 1;
  teamInfos[homeTeam].teamColor = TEAM_BLUE;
  teamInfos[homeTeam].messageBudget = 1200;
  teamInfos[awayTeam].teamNumber = 2;
  teamInfos[awayTeam].teamColor = TEAM_RED;
  teamInfos[awayTeam].messageBudget = 1200;
}

void GameController::registerOracle(int robot, Oracle& oracle) {
  ASSERT(!robots[robot].oracle);
  robots[robot].oracle = &oracle;
  if (fieldDimensions.xPosOwnPenaltyMark == 0.f) {
    fieldDimensions.load();
  }
}

bool GameController::handleGlobalCommand(const std::string& command) {
  Vector2<> ballPos;
  Oracle::getAbsoluteBallPosition(ballPos);
  if (command == "initial") {
    gameInfo.state = STATE_INITIAL;
    timeOfLastDropIn = 0;
    gameInfo.secsRemaining = durationOfHalf;
    freeKickBallContact = Pose2D(INFINITY, {0, 0});
    return true;
  } else if (command == "ready") {
    gameInfo.state = STATE_READY;
    for (int i = 0; i < numOfRobots; ++i) {
      if (robots[i].info.penalty) {
        handleRobotCommand(i, "none");
      }
    }
    timeWhenStateBegan = Time::getCurrentSystemTime();
    return true;
  } else if (command == "set") {
    gameInfo.state = STATE_SET;
    for (int i = 0; i < numOfRobots; ++i) {
      robots[i].info.penalty = none;
    }
    timeWhenStateBegan = Time::getCurrentSystemTime();
    if (gameInfo.setPlay != SET_PLAY_PENALTY_KICK) {
      Oracle::moveBall(Vector3<>(0.f, 0.f, 50.f), true);
    }
    return true;
  } else if (command == "playing") {
    gameInfo.state = STATE_PLAYING;
    timeWhenHalfStarted = Time::getCurrentSystemTime() - (durationOfHalf - gameInfo.secsRemaining) * 1000;
    return true;
  } else if (command == "finished") {
    gameInfo.state = STATE_FINISHED;
    return true;
  } else if (command == "normalPhase") {
    gameInfo.gamePhase = GAME_PHASE_NORMAL;
    gameInfo.state = STATE_INITIAL;
    return true;
  } else if (command == "penaltyPhase") {
    gameInfo.gamePhase = GAME_PHASE_PENALTYSHOOT;
    gameInfo.state = STATE_PLAYING;
    return true;
  } else if (command == "timeoutPhase") {
    gameInfo.gamePhase = GAME_PHASE_TIMEOUT;
    gameInfo.state = STATE_PLAYING;
    return true;
  } else if (command == "kickOffHome") {
    gameInfo.kickingTeam = 1;
    return true;
  } else if (command == "kickOffAway") {
    gameInfo.kickingTeam = 2;
    return true;
  } else if (command == "freeKickComplete") {
    gameInfo.setPlay = SET_PLAY_NONE;
    freeKickBallContact = Pose2D(INFINITY, {0, 0});
    return true;
  } else if (command == "cornerKickHome") {
    gameInfo.kickingTeam = 1;
    gameInfo.setPlay = SET_PLAY_CORNER_KICK;
    if (ballPos.y > 0) {
      Oracle::moveBall(Vector3<>(fieldDimensions.xPosOwnGroundline + 50.f, fieldDimensions.yPosLeftSideline - 50.f, 50.f),
                       true);
    } else {
      Oracle::moveBall(Vector3<>(fieldDimensions.xPosOwnGroundline + 50.f, fieldDimensions.yPosRightSideline + 50.f, 50.f),
                       true);
    }
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  } else if (command == "cornerKickAway") {
    gameInfo.kickingTeam = 2;
    gameInfo.setPlay = SET_PLAY_CORNER_KICK;
    if (ballPos.y > 0) {
      Oracle::moveBall(
        Vector3<>(fieldDimensions.xPosOpponentGroundline - 50.f, fieldDimensions.yPosLeftSideline - 50.f, 50.f), true);
    } else {
      Oracle::moveBall(
        Vector3<>(fieldDimensions.xPosOpponentGroundline - 50.f, fieldDimensions.yPosRightSideline + 50.f, 50.f), true);
    }
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  } else if (command == "goalFreeKickHome") {
    gameInfo.kickingTeam = 1;
    gameInfo.setPlay = SET_PLAY_GOAL_KICK;
    if (ballPos.y > 0) {
      Oracle::moveBall(Vector3<>(fieldDimensions.xPosOpponentGoalBox, fieldDimensions.yPosLeftGoalBox, 50.f), true);
    } else {
      Oracle::moveBall(Vector3<>(fieldDimensions.xPosOpponentGoalBox, fieldDimensions.yPosRightGoalBox, 50.f), true);
    }
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  } else if (command == "goalFreeKickAway") {
    gameInfo.kickingTeam = 2;
    gameInfo.setPlay = SET_PLAY_GOAL_KICK;
    if (ballPos.y > 0) {
      Oracle::moveBall(Vector3<>(fieldDimensions.xPosOwnGoalBox, fieldDimensions.yPosLeftGoalBox, 50.f), true);
    } else {
      Oracle::moveBall(Vector3<>(fieldDimensions.xPosOwnGoalBox, fieldDimensions.yPosRightGoalBox, 50.f), true);
    }
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  } else if (command == "kickInHome") {
    gameInfo.kickingTeam = 1;
    gameInfo.setPlay = SET_PLAY_KICK_IN;
    if (ballPos.y > 0) {
      Oracle::moveBall(Vector3<>(ballPos.x, fieldDimensions.yPosLeftSideline - 50.f, 50.f), true);
    } else {
      Oracle::moveBall(Vector3<>(ballPos.x, fieldDimensions.yPosRightSideline + 50.f, 50.f), true);
    }
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  } else if (command == "kickInAway") {
    gameInfo.kickingTeam = 2;
    gameInfo.setPlay = SET_PLAY_KICK_IN;
    if (ballPos.y > 0) {
      Oracle::moveBall(Vector3<>(ballPos.x, fieldDimensions.yPosLeftSideline - 50.f, 50.f), true);
    } else {
      Oracle::moveBall(Vector3<>(ballPos.x, fieldDimensions.yPosRightSideline + 50.f, 50.f), true);
    }
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  } else if (command == "freeKickHome") {
    gameInfo.kickingTeam = 1;
    gameInfo.setPlay = SET_PLAY_PUSHING_FREE_KICK;
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  } else if (command == "freeKickAway") {
    gameInfo.kickingTeam = 2;
    gameInfo.setPlay = SET_PLAY_PUSHING_FREE_KICK;
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  } else if (command == "penaltyKickHome") {
    gameInfo.kickingTeam = 1;
    gameInfo.setPlay = SET_PLAY_PENALTY_KICK;
    gameInfo.state = STATE_READY;
    Oracle::moveBall(Vector3<>(fieldDimensions.xPosOwnPenaltyMark, 0.f, 50.f), true);
    timeWhenStateBegan = Time::getCurrentSystemTime();
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  } else if (command == "penaltyKickAway") {
    gameInfo.kickingTeam = 2;
    gameInfo.setPlay = SET_PLAY_PENALTY_KICK;
    gameInfo.state = STATE_READY;
    Oracle::moveBall(Vector3<>(fieldDimensions.xPosOpponentPenaltyMark, 0.f, 50.f), true);
    timeWhenStateBegan = Time::getCurrentSystemTime();
    freeKickBallContact = Oracle::getLastBallContactRobot();
    return true;
  }

  return false;
}

bool GameController::handleGlobalConsole(In& stream) {
  SYNC;
  std::string command;
  stream >> command;
  return handleGlobalCommand(command);
}

bool GameController::handleRobotCommand(int robot, const std::string& command) {
  Robot& r = robots[robot];
  for (int i = 0; i < numOfPenalties; ++i) {
    if (command == getName((Penalty)i)) {
      r.info.penalty = i == manual ? PENALTY_MANUAL : (uint8_t)i;
      if (i) {
        r.timeWhenPenalized = Time::getCurrentSystemTime();
        if (automatic) {
          placeForPenalty(
            robot, fieldDimensions.xPosOpponentPenaltyMark, fieldDimensions.yPosRightFieldBorder + 100.f, -pi_2);
        }
      } else if (automatic) {
        ASSERT(r.oracle);
        Vector2<> ballPos;
        r.oracle->getAbsoluteBallPosition(ballPos);
        if (robot < numOfRobots / 2) {
          ballPos = -ballPos;
        }
        placeForPenalty(robot,
                        fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y >= 0 ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline,
                        ballPos.y >= 0 ? pi_2 : -pi_2);
      }
      return true;
    }
  }
  return false;
}

bool GameController::handleRobotConsole(int robot, In& stream) {
  SYNC;
  std::string command;
  stream >> command;
  return handleRobotCommand(robot, command);
}

void GameController::placeForPenalty(int robot, float x, float y, float rotation) {
  Robot& r = robots[robot];
  ASSERT(r.oracle);
  Vector2<> newPos(robot < numOfRobots / 2 ? x : -x, y);
  for (;;) {
    int j = 0;
    while (j < numOfRobots && (j == robot || !robots[j].oracle || (robots[j].lastPose.translation - newPos).abs() >= 300)) {
      ++j;
    }
    if (j == numOfRobots) {
      r.lastPose = Pose2D(rotation, newPos.x, newPos.y);
      r.oracle->moveRobot(Vector3<>(newPos.x, newPos.y, dropHeight), Vector3<>(0, 0, rotation), true);
      break;
    } else {
      newPos.x += newPos.x < 0 ? -400 : 400;
    }
  }
}

bool GameController::inOwnPenaltyArea(int robot) const {
  const Robot& r = robots[robot];
  if (r.lastPose.translation.y < fieldDimensions.yPosRightPenaltyArea ||
      r.lastPose.translation.y > fieldDimensions.yPosLeftPenaltyArea) {
    return false;
  } else if (robot < numOfRobots / 2) {
    return r.lastPose.translation.x >= fieldDimensions.xPosOpponentPenaltyArea &&
           (r.lastPose.translation.x <= fieldDimensions.xPosOpponentGroundline ||
            (r.lastPose.translation.x <= fieldDimensions.xPosOpponentGoal &&
             r.lastPose.translation.y >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y <= fieldDimensions.yPosLeftGoal));
  } else {
    return r.lastPose.translation.x <= fieldDimensions.xPosOwnPenaltyArea &&
           (r.lastPose.translation.x >= fieldDimensions.xPosOwnGroundline ||
            (r.lastPose.translation.x >= fieldDimensions.xPosOwnGoal &&
             r.lastPose.translation.y >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y <= fieldDimensions.yPosLeftGoal));
  }
}

void GameController::placeGoalie(int robot) {
  Robot& r = robots[robot];
  r.manuallyPlaced = r.oracle && !inOwnPenaltyArea(robot);
  if (r.manuallyPlaced) {
    r.lastPose = robot < numOfRobots / 2 ? Pose2D(-pi, fieldDimensions.xPosOpponentGroundline, 0)
                                         : Pose2D(0, fieldDimensions.xPosOwnGroundline, 0);
  }
}

void GameController::placeFromSet(int robot, int minRobot, const Pose2D* poses, int numOfPoses) {
  ASSERT(numOfPoses <= numOfFieldPlayers);

  // For finding a manual placement pose, it is determined which
  // of the positions would be chosen by our teammates.
  bool occupied[numOfFieldPlayers] = {false};
  for (int i = minRobot; i < minRobot + numOfFieldPlayers; ++i) {
    if (i != robot && robots[i].oracle) {
      const Robot& r2 = robots[i];
      float minDistance = std::numeric_limits<float>::max();
      int bestPoseIndex = 0;
      for (int j = 0; j < numOfPoses; ++j) {
        const Pose2D& pose = poses[j];
        float distance = (pose.translation - r2.lastPose.translation).abs();
        if (!occupied[j] && distance < minDistance) {
          minDistance = distance;
          bestPoseIndex = j;
        }
      }
      occupied[bestPoseIndex] = true;
    }
  }

  // The position that would not be chosen is suitable for this robot.
  int i = 0;
  while (i < numOfPoses && occupied[i]) {
    ++i;
  }
  ASSERT(i < numOfFieldPlayers);
  robots[robot].lastPose = poses[i];
}

void GameController::placeOffensivePlayers(int minRobot) {
  static const Pose2D poses[2][numOfFieldPlayers] = {
    {Pose2D(0, -fieldDimensions.centerCircleRadius - footLength, 0),
     Pose2D(0, fieldDimensions.xPosOwnPenaltyMark, fieldDimensions.yPosRightGoal),
     Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftPenaltyArea),
     Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightPenaltyArea)},
    {Pose2D(-pi, fieldDimensions.centerCircleRadius + footLength, 0),
     Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyMark, fieldDimensions.yPosLeftGoal),
     Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftPenaltyArea),
     Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightPenaltyArea)}};

  // Move all field players that are not in their own half or in their penalty area.
  // Count robots in center circle
  int numOfRobotsInCenterCircle = 0;
  float minDistance = std::numeric_limits<float>::max();
  int indexOfMinDistance = 0;
  for (int i = minRobot; i < minRobot + numOfFieldPlayers; ++i) {
    Robot& r = robots[i];
    r.manuallyPlaced = false;
    r.manuallyPlaced =
      r.oracle && (inOwnPenaltyArea(i) || r.lastPose.translation.y < fieldDimensions.yPosRightSideline ||
                   r.lastPose.translation.y > fieldDimensions.yPosLeftSideline ||
                   (i < numOfRobots / 2 && (r.lastPose.translation.x < footLength ||
                                            r.lastPose.translation.x > fieldDimensions.xPosOpponentGroundline)) ||
                   (i >= numOfRobots / 2 && (r.lastPose.translation.x > -footLength ||
                                             r.lastPose.translation.x < fieldDimensions.xPosOwnGroundline)));
    if (r.manuallyPlaced) {
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0], numOfFieldPlayers);
    } else {
      float distance = r.lastPose.translation.abs();
      if (distance < fieldDimensions.centerCircleRadius + footLength) {
        ++numOfRobotsInCenterCircle;
        if (distance < minDistance) {
          minDistance = distance;
          indexOfMinDistance = i;
        }
      }
    }
  }

  // If there is more than one robot in center circle, keep the closest
  // to the field center and move the others away.
  if (numOfRobotsInCenterCircle > 1) {
    for (int i = minRobot; i < minRobot + numOfFieldPlayers; ++i) {
      Robot& r = robots[i];
      if (i != indexOfMinDistance && r.lastPose.translation.abs() < fieldDimensions.centerCircleRadius + 120.f) {
        placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0] + 1, numOfFieldPlayers - 1);
      }
    }
  }
}

void GameController::placeDefensivePlayers(int minRobot) {
  static const Pose2D poses[2][numOfFieldPlayers] = {
    {Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
     Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightGoal / 2.f),
     Pose2D(0,
            fieldDimensions.xPosOwnPenaltyArea + safeDistance,
            (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f),
     Pose2D(0,
            fieldDimensions.xPosOwnPenaltyArea + safeDistance,
            (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f)},
    {Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
     Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightGoal / 2.f),
     Pose2D(-pi,
            fieldDimensions.xPosOpponentPenaltyArea - safeDistance,
            (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f),
     Pose2D(-pi,
            fieldDimensions.xPosOpponentPenaltyArea - safeDistance,
            (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f)}};
  for (int i = minRobot; i < minRobot + numOfFieldPlayers; ++i) {
    Robot& r = robots[i];
    r.manuallyPlaced =
      r.oracle && (inOwnPenaltyArea(i) || r.lastPose.translation.abs() < fieldDimensions.centerCircleRadius + footLength ||
                   r.lastPose.translation.y < fieldDimensions.yPosRightSideline ||
                   r.lastPose.translation.y > fieldDimensions.yPosLeftSideline ||
                   (i < numOfRobots / 2 && (r.lastPose.translation.x < footLength ||
                                            r.lastPose.translation.x > fieldDimensions.xPosOpponentGroundline)) ||
                   (i >= numOfRobots / 2 && (r.lastPose.translation.x > -footLength ||
                                             r.lastPose.translation.x < fieldDimensions.xPosOwnGroundline)));
    if (r.manuallyPlaced) {
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0], numOfFieldPlayers);
    }
  }
}

void GameController::executePlacement() {
  for (int i = 0; i < numOfRobots; ++i) {
    const Robot& r = robots[i];
    if (r.manuallyPlaced) {
      r.oracle->moveRobot(Vector3<>(r.lastPose.translation.x, r.lastPose.translation.y, dropHeight),
                          Vector3<>(0, 0, r.lastPose.rotation),
                          true);
    }
  }
}

void GameController::referee() {
  if (automatic) {
    SYNC;
    switch (gameInfo.state) {
    case STATE_READY:
      if (Time::getTimeSince(timeWhenStateBegan) < 2000) {
        timeWhenLastRobotMoved = 0;
      }
      if (Time::getTimeSince(timeWhenStateBegan) >= 45000 ||
          (timeWhenLastRobotMoved && Time::getTimeSince(timeWhenLastRobotMoved) > 2000)) {
        handleGlobalCommand("set");
      }
      break;

    case STATE_SET:
      if (Time::getTimeSince(timeWhenStateBegan) >= 5000) {
        handleGlobalCommand("playing");
      }
      break;

    case STATE_PLAYING:
      switch (Oracle::updateBall()) {
      case Oracle::GOAL_BY_HOME:
        ++teamInfos[homeTeam].score;
        VERIFY(handleGlobalCommand("kickOffAway"));
        VERIFY(handleGlobalCommand("ready"));
        break;
      case Oracle::GOAL_BY_AWAY:
        ++teamInfos[awayTeam].score;
        VERIFY(handleGlobalCommand("kickOffHome"));
        VERIFY(handleGlobalCommand("ready"));
        break;
      case Oracle::OUT_BY_HOME:
        VERIFY(handleGlobalCommand("kickInAway"));
        break;
      case Oracle::OUT_BY_AWAY:
        VERIFY(handleGlobalCommand("kickInHome"));
        break;
      case Oracle::CORNER_KICK_HOME:
        VERIFY(handleGlobalCommand("cornerKickHome"));
        break;
      case Oracle::CORNER_KICK_AWAY:
        VERIFY(handleGlobalCommand("cornerKickAway"));
        break;
      case Oracle::GOAL_FREE_KICK_HOME:
        VERIFY(handleGlobalCommand("goalFreeKickHome"));
        break;
      case Oracle::GOAL_FREE_KICK_AWAY:
        VERIFY(handleGlobalCommand("goalFreeKickAway"));
        break;
      case Oracle::NONE:
        break;
      }
      Pose2D pose = Oracle::getLastBallContactRobot();
      if (freeKickBallContact.rotation != INFINITY && Oracle::getLastBallContactRobot() != freeKickBallContact) {
        VERIFY(handleGlobalCommand("freeKickComplete"));
      }
    }
  }
}

void GameController::writeGameInfo(Out& stream) {
  SYNC;
  if (gameInfo.state == STATE_PLAYING) {
    gameInfo.secsRemaining = (uint16_t)(durationOfHalf - Time::getTimeSince(timeWhenHalfStarted) / 1000);
  }
  gameInfo.timeLastPackageReceived = Time::getCurrentSystemTime();
  stream << gameInfo;
}

void GameController::writeOwnTeamInfo(int robot, Out& stream) {
  SYNC;
  stream << teamInfos[robot * 2 / numOfRobots];
}

void GameController::writeOpponentTeamInfo(int robot, Out& stream) {
  SYNC;
  stream << teamInfos[1 - robot * 2 / numOfRobots];
}

void GameController::writeRobotInfo(int robot, Out& stream) {
  SYNC;
  Robot& r = robots[robot];
  if (r.info.penalty) {
    r.info.secsTillUnpenalised = (uint8_t)(std::max(int(30 - Time::getTimeSince(r.timeWhenPenalized) / 1000), 0));
  }
  RobotPose pose;
  ASSERT(r.oracle);
  r.oracle->getRobotPose(pose);
  if (robot < numOfRobots / 2) {
    (Pose2D&)pose = Pose2D(pi) + pose;
  }
  if ((pose.translation - r.lastPose.translation).abs() > 5 || normalize(pose.rotation - r.lastPose.rotation) > 0.05) {
    timeWhenLastRobotMoved = Time::getCurrentSystemTime();
    r.lastPose = pose;
  }
  stream << r.info;
}

void GameController::addCompletion(std::set<std::string>& completion) const {
  static const char* commands[] = {"initial",
                                   "ready",
                                   "set",
                                   "playing",
                                   "finished",
                                   "kickOffHome",
                                   "kickOffAway",
                                   "freeKickComplete",
                                   "cornerKickHome",
                                   "cornerKickAway",
                                   "goalFreeKickHome",
                                   "goalFreeKickAway",
                                   "kickInHome",
                                   "kickInAway",
                                   "freeKickHome",
                                   "freeKickAway"};
  const int num = sizeof(commands) / sizeof(commands[0]);
  for (int i = 0; i < num; ++i) {
    completion.insert(std::string("gc ") + commands[i]);
  }
  for (int i = 0; i < numOfPenalties; ++i) {
    if (i != obstruction) {
      completion.insert(std::string("pr ") + getName((Penalty)i));
    }
  }
}
