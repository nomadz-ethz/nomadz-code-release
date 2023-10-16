/**
 * @file FieldPositionProvider.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "FieldPositionProvider.h"

MAKE_MODULE(FieldPositionProvider, Behavior Control);

FieldPositionProvider::FieldPositionProvider() {}

void FieldPositionProvider::update(FieldPosition& fieldPosition) {
  fieldPosition.readyPose = denormalizePose(getReadyPose());
  fieldPosition.standByPose = denormalizePose(getStandByPose(fieldPosition));
  fieldPosition.searchPose = denormalizePose(getSearchPose(useOptimalPosition));
  fieldPosition.currentTargetPose = getCurrentActivePose();
}

Pose2D FieldPositionProvider::denormalizePose(const Pose2D& pose) {
  return Pose2D(pose.rotation,
                pose.translation.x * theFieldDimensions.xPosOpponentGroundline,
                pose.translation.y * theFieldDimensions.yPosLeftSideline);
}

Pose2D FieldPositionProvider::normalizePose(const Pose2D& pose) {
  return Pose2D(pose.rotation,
                pose.translation.x / theFieldDimensions.xPosOpponentGroundline,
                pose.translation.y / theFieldDimensions.yPosLeftSideline);
}

Pose2D FieldPositionProvider::getCurrentActivePose() {
  Pose2D currentTargetPose;
  Vector2<> robotPose = theRobotPose.translation;

  float step = 500; // in mm

  std::vector<float> score(9);    // score list: C, N, NE, E, SE, S, SW, W, NW
  std::vector<float> gradient(8); // no Center
  Vector2<> transVec;

  for (int i = 0; i < 9; i++) {
    switch (i) {
    case 0:
      transVec = robotPose;
      break;
    case 1:
      transVec = Vector2<>(robotPose.x + step, robotPose.y);
      break;
    case 2:
      transVec = Vector2<>(robotPose.x + step, robotPose.y - step);
      break;
    case 3:
      transVec = Vector2<>(robotPose.x, robotPose.y - step);
      break;
    case 4:
      transVec = Vector2<>(robotPose.x - step, robotPose.y - step);
      break;
    case 5:
      transVec = Vector2<>(robotPose.x - step, robotPose.y);
      break;
    case 6:
      transVec = Vector2<>(robotPose.x - step, robotPose.y + step);
      break;
    case 7:
      transVec = Vector2<>(robotPose.x, robotPose.y + step);
      break;
    case 8:
      transVec = Vector2<>(robotPose.x + step, robotPose.y + step);
      break;
    }

    score[i] = coverageIntegral(transVec) + ballCoM(transVec) + avoidanceGaussian(transVec) + awayFromGoal(transVec);
  }

  for (int i = 0; i < 8; i++) {
    gradient[i] = score[i + 1] - score[0];
  }

  auto minGradIt = std::min_element(gradient.begin(), gradient.end()); // it's negative
  float minGrad = *minGradIt;
  auto direction = std::distance(gradient.begin(), minGradIt);

  float translation = 3. * abs(minGrad);
  float rotation = (theCombinedWorldModel.ballState.position - theRobotPose.translation).angle();

  switch (direction) {
  case (0):
    currentTargetPose = Pose2D(rotation, robotPose.x + translation, robotPose.y);
    break;
  case (1):
    currentTargetPose = Pose2D(rotation, robotPose.x + translation, robotPose.y - translation);
    break;
  case (2):
    currentTargetPose = Pose2D(rotation, robotPose.x, robotPose.y - translation);
    break;
  case (3):
    currentTargetPose = Pose2D(rotation, robotPose.x - translation, robotPose.y - translation);
    break;
  case (4):
    currentTargetPose = Pose2D(rotation, robotPose.x - translation, robotPose.y);
    break;
  case (5):
    currentTargetPose = Pose2D(rotation, robotPose.x - translation, robotPose.y + translation);
    break;
  case (6):
    currentTargetPose = Pose2D(rotation, robotPose.x, robotPose.y + translation);
    break;
  case (7):
    currentTargetPose = Pose2D(rotation, robotPose.x + translation, robotPose.y + translation);
    break;
  }

  return currentTargetPose;
}

Pose2D FieldPositionProvider::getReadyPose() {
  Pose2D startPose;
  if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK) {
    switch (theBehaviorStatus.role) {
    case BehaviorStatus::keeper:
      if (theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam) {
        startPose = Pose2D(0.f, setPosPenaltyKeeper);
      } else {
        startPose = Pose2D(0.f, setPosKeeper);
      }
      break;
    case BehaviorStatus::striker:
      if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
        startPose = penaltyStrikerPosition(0.f, getOtherActiveStrikers(theRobotInfo.number));
      } else {
        startPose = penaltyStrikerPosition(3.14f, getOtherActiveStrikers(theRobotInfo.number));
      }
      break;
    case BehaviorStatus::supporter:
      if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
        startPose = penaltySupporterPosition(0.f, getOtherActiveSupporters(theRobotInfo.number));
      } else {
        startPose = penaltySupporterPosition(3.14f, getOtherActiveSupporters(theRobotInfo.number));
      }
      break;
    case BehaviorStatus::defender:
      if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
        startPose = Pose2D(0.f, (theRobotInfo.number % 2 == 0) ? setPosPenaltyDefenderEven : setPosPenaltyDefenderOdd);
      } else {
        startPose = Pose2D(3.14f, (theRobotInfo.number % 2 == 0) ? setPosPenaltyDefenderEven : setPosPenaltyDefenderOdd);
      }
      break;
    default:
      startPose = Pose2D(0.f, setPosKickoffOther);
      break;
    }
  } else {
    if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
      switch (theBehaviorStatus.role) {
      case BehaviorStatus::keeper:
        startPose = Pose2D(0.f, setPosKickoffKeeper);
        break;
      case BehaviorStatus::striker:
        startPose = assignKickoffStrikerPosition(getOtherActiveStrikers(theRobotInfo.number), getOtherActiveSupporters(0));
        break;
      case BehaviorStatus::supporter:
        startPose = assignKickoffSupporterPosition(getOtherActiveSupporters(theRobotInfo.number));
        break;
      case BehaviorStatus::defender:
        startPose = assignKickoffDefenderPosition(getOtherActiveDefenders(theRobotInfo.number));
        break;
      default:
        startPose = Pose2D(0.f, setPosKickoffOther);
        break;
      }
    } else {
      switch (theBehaviorStatus.role) {
      case BehaviorStatus::keeper:
        startPose = Pose2D(0.f, setPosKeeper);
        break;
      case BehaviorStatus::striker:
        startPose = assignStrikerPosition(getOtherActiveStrikers(theRobotInfo.number));
        break;
      case BehaviorStatus::supporter:
        startPose = assignSupporterPosition(getOtherActiveSupporters(theRobotInfo.number));
        break;
      case BehaviorStatus::defender:
        startPose = assignDefenderPosition(getOtherActiveDefenders(theRobotInfo.number));
        break;
      default:
        startPose = Pose2D(0.f, setPosOther);
        break;
      }
    }
  }
  return startPose;
}

Pose2D FieldPositionProvider::getSearchPose(bool optimal) {
  Pose2D searchPose;
  if (optimal) {
    // use logic with GD
    ;
  } else {
    int scale = 0;
    int total = 0;
    float posX = 0.;
    float posY = 0.;

    float defenceRadiusX = 0.3;
    float defenceRadiusY = 0.5;
    float attackRadiusX = 0.35;
    float attackRadiusY = 0.6;

    switch (theBehaviorStatus.role) {
    case BehaviorStatus::keeper:
      searchPose = Pose2D(0.f, -0.95, 0.);
      break;
    case BehaviorStatus::defender:
      total = getOtherActiveDefenders(0) + 1;
      scale = getOtherActiveDefenders(theRobotInfo.number);
      posX = -cos(pi2 / (total + 1) * (total - scale)) * defenceRadiusX - (1 - defenceRadiusX);
      posY = sin(pi2 / (total + 1) * (total - scale)) * defenceRadiusY;
      searchPose = Pose2D(0., posX, posY);
      break;
    case BehaviorStatus::supporter:
      if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber) {
        int numOtherPlayers = getOtherActiveSupporters(0);
        int other_players = getOtherActiveSupporters(theRobotInfo.number);
        float fraction = (numOtherPlayers != 0) ? (other_players) / ((float)numOtherPlayers) : 0.5;
        posX = theFieldDimensions.xPosOpponentPenaltyArea;
        posY = (theFieldDimensions.yPosLeftSideline + theFieldDimensions.yPosLeftGoalBox) / 2.0 * fraction +
               (theFieldDimensions.yPosRightSideline + theFieldDimensions.yPosRightGoalBox) / 2.0 * (1 - fraction);
        searchPose = normalizePose(Pose2D(0., posX, posY));
      } else {
        total = getOtherActiveSupporters(0) + getOtherActiveStrikers(0); // it would not consider the robot itself though
        scale = getOtherActiveSupporters(theRobotInfo.number) + getOtherActiveStrikers(theRobotInfo.number);
        posX = -cos(pi2 / (total + 1) * (total - scale)) * attackRadiusX + attackRadiusX;
        posY = -sin(pi2 / (total + 1) * (total - scale)) * attackRadiusY;
        searchPose = Pose2D(0., posX, posY);
      }
      break;
    case BehaviorStatus::striker:
      if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber) {
        int numOtherPlayers = getOtherActiveStrikers(0);
        int other_players = getOtherActiveStrikers(theRobotInfo.number);
        float fraction = (numOtherPlayers != 0) ? (other_players) / ((float)numOtherPlayers) : 0.5;
        posX = theFieldDimensions.xPosOpponentGoalBox;
        posY = (theFieldDimensions.yPosLeftSideline + theFieldDimensions.yPosLeftPenaltyArea) / 2.0 * fraction +
               (theFieldDimensions.yPosRightSideline + theFieldDimensions.yPosRightPenaltyArea) / 2.0 * (1 - fraction);
        searchPose = normalizePose(Pose2D(0., posX, posY));
      } else {
        total = getOtherActiveSupporters(0) + getOtherActiveStrikers(0);
        scale = getOtherActiveSupporters(theRobotInfo.number) + getOtherActiveStrikers(theRobotInfo.number);
        posX = -cos(pi2 / (total + 1) * (total - scale)) * attackRadiusX + attackRadiusX;
        posY = -sin(pi2 / (total + 1) * (total - scale)) * attackRadiusY;
        searchPose = Pose2D(0., posX, posY);
      }
      break;
    default:
      searchPose = Pose2D(0., 0., 0.);
      break;
    }
  }

  return searchPose;
}

Pose2D FieldPositionProvider::getStandByPose(FieldPosition& fieldPosition) {
  Pose2D standByPose;
  switch (theBehaviorStatus.role) {
  case BehaviorStatus::defender:
    standByPose = getDefenderStandByPose();
    // standByPose = fieldPosition.readyPose;
    break;
  default:
    standByPose = fieldPosition.readyPose;
    break;
  }
  return standByPose;
}

int FieldPositionProvider::getOtherActiveStrikers(int ownNumber) {
  int strikers = 0;
  for (int i = ownNumber + 1; i < TeamMateData::numOfPlayers; i++) { // the role of the robot itslef is dummy
    if ((theTeamMateData.behaviorStatus[i].role == BehaviorStatus::striker) &&
        (!theTeamMateData.isPenalized[i])) { // should consider active, but it causes problems during ready.
      strikers += 1;
    }
  }
  return strikers;
}

int FieldPositionProvider::getOtherActiveSupporters(int ownNumber) {
  int supporters = 0;
  for (int i = ownNumber + 1; i < TeamMateData::numOfPlayers; i++) {
    if ((theTeamMateData.behaviorStatus[i].role == BehaviorStatus::supporter) &&
        (!theTeamMateData.isPenalized[i])) { // the role of the robot itslef is dummy
      supporters += 1;
    }
  }
  return supporters;
}

int FieldPositionProvider::getOtherActiveDefenders(int ownNumber) {
  int defenders = 0;
  for (int i = ownNumber + 1; i < TeamMateData::numOfPlayers; i++) {
    if ((theTeamMateData.behaviorStatus[i].role == BehaviorStatus::defender) &&
        (!theTeamMateData.isPenalized[i])) { // the role of the robot itslef is dummy
      defenders += 1;
    }
  }
  return defenders;
}

Pose2D FieldPositionProvider::assignStrikerPosition(int otherActStriker) {
  Pose2D assignedPos;

  if (otherActStriker == 0)
    assignedPos = Pose2D(0.f, setPosStriker);
  else if (otherActStriker == 1)
    assignedPos = Pose2D(0.f, setPosStrikerOne);
  else if (otherActStriker == 2)
    assignedPos = Pose2D(0.f, setPosStrikerTwo);
  else if (otherActStriker == 3)
    assignedPos = Pose2D(0.f, setPosStrikerThree);
  else if (otherActStriker == 4)
    assignedPos = Pose2D(0.f, setPosStrikerFour);
  else
    assignedPos = Pose2D(0.f, setPosOther);
  return assignedPos;
}

Pose2D FieldPositionProvider::assignKickoffStrikerPosition(int otherActStriker, int actSupporter) {
  Pose2D assignedPos;

  if (actSupporter == 0 && otherActStriker == 0) {
    assignedPos = Pose2D(0.f, setPosKickoffSupporter);
    return assignedPos;
  }

  if (otherActStriker == 0)
    assignedPos = Pose2D(0.f, setPosKickoffStriker);
  else if (otherActStriker == 1)
    assignedPos = Pose2D(0.f, setPosKickoffStrikerOne);
  else if (otherActStriker == 2)
    assignedPos = Pose2D(0.f, setPosKickoffStrikerTwo);
  else if (otherActStriker == 3)
    assignedPos = Pose2D(0.f, setPosKickoffStrikerThree);
  else if (otherActStriker == 4)
    assignedPos = Pose2D(0.f, setPosKickoffStrikerFour);
  else
    assignedPos = Pose2D(0.f, setPosOther);
  return assignedPos;
}

Pose2D FieldPositionProvider::assignSupporterPosition(int otherActSupporter) {
  Pose2D assignedPos;

  if (otherActSupporter == 0)
    assignedPos = Pose2D(0.f, setPosSupporter);
  else if (otherActSupporter == 1)
    assignedPos = Pose2D(0.f, setPosSupporterOne);
  else if (otherActSupporter == 2)
    assignedPos = Pose2D(0.f, setPosSupporterTwo);
  else if (otherActSupporter == 3)
    assignedPos = Pose2D(0.f, setPosSupporterThree);
  else if (otherActSupporter == 4)
    assignedPos = Pose2D(0.f, setPosSupporterFour);
  else
    assignedPos = Pose2D(0.f, setPosOther);
  return assignedPos;
}

Pose2D FieldPositionProvider::assignKickoffSupporterPosition(int otherActSupporter) {
  Pose2D assignedPos;

  if (otherActSupporter == 0)
    assignedPos = Pose2D(0.f, setPosKickoffSupporter);
  else if (otherActSupporter == 1)
    assignedPos = Pose2D(0.f, setPosKickoffSupporterOne);
  else if (otherActSupporter == 2)
    assignedPos = Pose2D(0.f, setPosKickoffSupporterTwo);
  else if (otherActSupporter == 3)
    assignedPos = Pose2D(0.f, setPosKickoffSupporterThree);
  else if (otherActSupporter == 4)
    assignedPos = Pose2D(0.f, setPosKickoffSupporterFour);
  else
    assignedPos = Pose2D(0.f, setPosOther);
  return assignedPos;
}

Pose2D FieldPositionProvider::assignDefenderPosition(int otherActDefender) {
  Pose2D assignedPos;

  if (otherActDefender == 0)
    assignedPos = Pose2D(0.f, setPosDefender);
  else if (otherActDefender == 1)
    assignedPos = Pose2D(0.f, setPosDefenderOne);
  else if (otherActDefender == 2)
    assignedPos = Pose2D(0.f, setPosDefenderTwo);
  // assumes up to three defenders
  else
    assignedPos = Pose2D(0.f, setPosOther);
  return assignedPos;
}

Pose2D FieldPositionProvider::assignKickoffDefenderPosition(int otherActDefender) {
  Pose2D assignedPos;

  if (otherActDefender == 0)
    assignedPos = Pose2D(0.f, setPosKickoffDefender);
  else if (otherActDefender == 1)
    assignedPos = Pose2D(0.f, setPosKickoffDefenderOne);
  else if (otherActDefender == 2)
    assignedPos = Pose2D(0.f, setPosKickoffDefenderTwo);
  // assumes up to three defenders
  else
    assignedPos = Pose2D(0.f, setPosOther);
  return assignedPos;
}

Pose2D FieldPositionProvider::penaltyStrikerPosition(float rot, int otherStriker) {
  Vector2<> d = penaltyStrikerDx * ((otherStriker + 1) / 2) +
                penaltyStrikerDy * pow((-1), (otherStriker % 2 + 1)) * ((otherStriker + 1) / 2);
  if (theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam) {
    d -= Vector2<>(0.5, 0);
  }
  Pose2D assignedPos = Pose2D(rot, setPosPenaltyStriker + d);

  return assignedPos;
}

Pose2D FieldPositionProvider::penaltySupporterPosition(float rot, int otherSupporter) {
  Vector2<> d = penaltySupporterDx * ((otherSupporter + 1) / 2) +
                penaltySupporterDy * pow((-1), (otherSupporter % 2)) * ((otherSupporter + 1) / 2);
  if (theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam) {
    d -= Vector2<>(0.5, 0);
  }
  Pose2D assignedPos = Pose2D(rot, setPosPenaltySupporter + d);

  return assignedPos;
}

float FieldPositionProvider::avoidanceGaussian(Vector2<> ownPos) {

  float error = 0.;
  float value = 0.;
  float scale = 0.;
  float variance = 1.;
  bool teamData = false;
  Vector2<> obsPos;

  for (int i = 0; i < thePlayerModel.players.size(); i++) {

    obsPos = theRobotPose * thePlayerModel.players[i].relPosOnField; // convert to global position
    teamData = !thePlayerModel.players[i].opponent;
    scale = (teamData) ? 150. : 250.;  // larger for opponents
    variance = (teamData) ? 0.5 : 0.5; // need smaller variance than coverage
    error = pow((ownPos.x - obsPos.x) / 1000., 2) + pow((ownPos.y - obsPos.y) / 1000., 2);
    value += scale * exp(-error / variance);
  }

  return value;
}

float FieldPositionProvider::coverageIntegral(Vector2<> ownPos) {

  float scale = 5.;
  float variance = 1.; // variance must be small using meters

  if (ownPos.x > theFieldDimensions.xPosOpponentGroundline || ownPos.x < theFieldDimensions.xPosOwnGroundline ||
      ownPos.y > theFieldDimensions.yPosLeftSideline || ownPos.y < theFieldDimensions.yPosRightSideline) {
    return 0.;
  }

  float value = 0.;
  float x, y, error1, error2;

  for (int i = 0; i < TeamMateData::numOfPlayers; i++) {
    if (!theTeamMateData.isActive[i]) {
      continue;
    }
    for (x = -1000; x < 1001; x += 200) {
      if (ownPos.x + x > theFieldDimensions.xPosOpponentGoalBox ||
          ownPos.x + x < theFieldDimensions.xPosOwnPenaltyArea) { // no reward for outside the field
        continue;
      }
      for (y = -1000; y < 1001; y += 200) {
        if (ownPos.y + y > 1.2 * theFieldDimensions.yPosLeftPenaltyArea ||
            ownPos.y + y < 1.2 * theFieldDimensions.yPosRightPenaltyArea) {
          continue;
        }
        error1 = pow(x / 1000., 2) + pow(y / 1000., 2);
        error2 = pow((ownPos.x + x - theTeamMateData.robotPoses[i].translation.x) / 1000., 2) +
                 pow((ownPos.y + y - theTeamMateData.robotPoses[i].translation.y) / 1000., 2);
        value += scale * exp(-error1 / variance) - 2 * scale * exp(-error2 / variance);
      }
    }
  }

  return -value;
}

float FieldPositionProvider::ballCoM(Vector2<> ownPos) {

  float scale = 200.; // 100 more conservative
  float x_tot = ownPos.x;
  float y_tot = ownPos.y;
  int active_robots = 1.;

  if (ownPos.x > theFieldDimensions.xPosOpponentGroundline || ownPos.x < theFieldDimensions.xPosOwnGroundline ||
      ownPos.y > theFieldDimensions.yPosLeftSideline || ownPos.y < theFieldDimensions.yPosRightSideline) {
    return 1e4;
  }

  for (int i = 0; i < TeamMateData::numOfPlayers; i++) {
    if (theTeamMateData.isActive[i] && (theTeamMateData.behaviorStatus[i].role == BehaviorStatus::supporter ||
                                        theTeamMateData.behaviorStatus[i].role == BehaviorStatus::striker)) {
      active_robots += 1.;
      x_tot += theTeamMateData.robotPoses[i].translation.x;
      y_tot += theTeamMateData.robotPoses[i].translation.y;
    }
  }

  float error =
    pow((x_tot / active_robots - std::min(theCombinedWorldModel.ballState.position.x + 1500,
                                          static_cast<float>(theFieldDimensions.xPosOpponentPenaltyArea))) /
          1000.,
        2) +
    pow((y_tot / active_robots - std::min(std::max(static_cast<float>(theCombinedWorldModel.ballState.position.y),
                                                   static_cast<float>(theFieldDimensions.yPosRightPenaltyArea)),
                                          static_cast<float>(theFieldDimensions.yPosLeftPenaltyArea))) /
          1000.,
        2); // we track 1.5m ahead of the ball, since the robot with ballock is there already

  return scale * error;
}

float FieldPositionProvider::awayFromGoal(Vector2<> ownPos) {

  if (theCombinedWorldModel.ballState.position.x < theFieldDimensions.centerCircleRadius ||
      ownPos.x < theCombinedWorldModel.ballState.position.x) { // need to compare with pass transition in EngageBall
    return 0.;
  }

  float scale = 1000.; // need to tune

  float t1 = (Vector2<>(theFieldDimensions.xPosOpponentGoal - theCombinedWorldModel.ballState.position.x,
                        theCombinedWorldModel.ballState.position.y))
               .angle();

  float t2 = (Vector2<>(theFieldDimensions.xPosOpponentGoal - ownPos.x, ownPos.y)).angle();

  float t3 = std::abs(t1 - t2);

  return -scale * t3;
}

Pose2D FieldPositionProvider::getDefenderStandByPose() {
  float radius = 0.6f;         // defender standby radius
  float clip_threshold = 25.f; // for clipping theta
  float delta = 50.f;          // angle between defenders
  Vector2<> gloBall = theCombinedWorldModel.ballState.position;
  float theta = atan2(gloBall.y, (gloBall.x + theFieldDimensions.xPosOpponentGroundline));
  float theta_mid;
  float pose_theta;

  // clip the theta between [-90 + threshold, 90 - thresdhold]
  if (theta > fromDegrees(90 - clip_threshold)) {
    theta_mid = fromDegrees(90 - clip_threshold);
  } else {
    theta_mid = theta;
  }
  if (theta_mid < (fromDegrees(-90 + clip_threshold))) {
    theta_mid = fromDegrees(-90 + clip_threshold);
  } else {
    theta_mid = theta;
  }

  if (getOtherActiveDefenders(theRobotInfo.number) == 0) {
    theta = theta_mid;
    pose_theta = theta;
  } else if (getOtherActiveDefenders(theRobotInfo.number) == 1) {
    theta = theta_mid - fromDegrees(delta / 2.);
    pose_theta = theta + fromDegrees(delta / 2.);
  } else if (getOtherActiveDefenders(theRobotInfo.number) == 2) {
    theta = theta_mid + fromDegrees(delta / 2.);
    pose_theta = theta - fromDegrees(delta / 2.);
  }

  Vector2<> standByPos = Vector2<>(cos(theta) * radius * 2 / 3 - 1.f, sin(theta) * radius);
  Pose2D standByPose = Pose2D(pose_theta, standByPos);
  return standByPose;
}
