/**
 * @file PlayerLocator.cpp
 *
 * Implementation of the PlayerLocator module
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "PlayerLocator.h"
#include "Core/Math/Matrix2x2.h"
#include <iostream>
#include <math.h>
#include <fstream>

MAKE_MODULE(PlayerLocator, Modeling)

PlayerLocator::PlayerLocator() {
  init = true;
}

void PlayerLocator::update(PlayerModel& playerModel) {
  DECLARE_DEBUG_DRAWING("module:PlayerLocator:players", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PlayerLocator:potentialPlayers", "drawingOnField");

  if (init) {
    playerModel.potentialPlayers.clear();
    init = false;
  }

  std::vector<PlayerModel::Player> potentialPlayers = playerModel.potentialPlayers;
  // Update position of potentialPlayers with odometry data
  for (auto& potentialPlayer : potentialPlayers) {
    potentialPlayer.relPosOnField = theOdometer.odometryOffset.invert() * potentialPlayer.relPosOnField;
    // FIXME: update covariance from motion
    potentialPlayer.detected = false;
  }
  // Update potentialPlayers given the new perceptions
  for (auto& perceptedPlayer : thePlayerPercept.players) {
    bool playerAlreadySeen = false;
    Vector2<> perceptedPlayerLoc = perceptedPlayer.centerOnField;
    perceptedPlayerLoc.x += perceptOffset;

    float angle = atan2(perceptedPlayerLoc.y, perceptedPlayerLoc.x);
    Matrix2x2<> perceptCovar(std::pow(70, 2), 0, 0, std::pow(50, 2));
    perceptCovar = perceptCovar.rotate(angle);
    for (auto& potentialPlayer : potentialPlayers) {
      Vector2<> dist = (potentialPlayer.relPosOnField - perceptedPlayerLoc);
      // if (dist * (potentialPlayer.covariance.invert() * dist) < 2) { // FIXME: this does not work well
      if (dist.abs() < mergeThreshold) {
        Matrix2x2<> covPlusSensorCov = perceptCovar + potentialPlayer.covariance;
        Matrix2x2<> k = potentialPlayer.covariance * covPlusSensorCov.invert();
        Vector2<> innovation = perceptedPlayerLoc - potentialPlayer.relPosOnField;
        potentialPlayer.relPosOnField += k * innovation;
        potentialPlayer.covariance -= k * potentialPlayer.covariance;
        potentialPlayer.minVariance =
          std::min(static_cast<float>(potentialPlayer.minVariance), std::sqrt(potentialPlayer.covariance.det()));

        potentialPlayer.timeStamp = theFrameInfo.time;
        potentialPlayer.detected = true;
        playerAlreadySeen = true;
        break;
      }
    }

    if (!playerAlreadySeen) { // Add new player to the list
      PlayerModel::Player pP;

      pP.relPosOnField = perceptedPlayerLoc;
      pP.opponent = perceptedPlayer.opponent;
      pP.standing = perceptedPlayer.standing;

      pP.covariance = perceptCovar;
      pP.minVariance = std::sqrt(pP.covariance.det());

      pP.timeStamp = theFrameInfo.time;
      pP.firstSeen = theFrameInfo.time;
      pP.detected = true;

      potentialPlayers.push_back(pP);
    }
  }

  // eliminate potential players
  auto it = potentialPlayers.begin();
  while (it != potentialPlayers.end()) {
    // increase uncertainty if the robot hasn't been detected in the current frame
    if (!it->detected) {
      it->covariance += Matrix2x2<>(pow(1, 2), 0, 0, pow(1, 2));
    }
    // potential player is too old or too much uncertainty
    auto timeDiff = theFrameInfo.time - it->timeStamp;
    if (timeDiff > rememberingTime) {
      it = potentialPlayers.erase(it);
    } else {
      // HOTFIX: eliminate dublicated entries generated when the head is moving, why are they around in the first place??
      auto it2 = next(it);
      while (it2 != potentialPlayers.end()) {
        if ((it->relPosOnField - it2->relPosOnField).abs() < mergeThreshold) {
          it->timeStamp = std::max(it->timeStamp, it2->timeStamp);
          it->firstSeen = std::min(it->firstSeen, it2->firstSeen);
          it2 = potentialPlayers.erase(it2);
        } else {
          it2++;
        }
      }
      it++;
    }
  }

  // Push potentialPlayer back to players vector
  playerModel.potentialPlayers.clear();
  for (auto& potentialPlayer : potentialPlayers) {
    playerModel.potentialPlayers.push_back(potentialPlayer);
  }
  playerModel.players.clear();

  for (auto& potentialPlayer : playerModel.potentialPlayers) {
    // Is potentialPlayer in field?
    if (isPlayerOutsideOfField(potentialPlayer) && outsideFieldCheck && theRobotPose.validity > 0.3) {
      continue;
    }

    // Has player been seen long enough?
    if ((potentialPlayer.timeStamp - potentialPlayer.firstSeen) < initTime) {
      continue;
    }

    // Did player reach low enough variance
    if (potentialPlayer.minVariance > minVariance) {
      continue;
    }

    // Is player the estimate of the relative position to far away for an accurate estimate?
    if (potentialPlayer.relPosOnField.abs() > distanceThreshold) {
      continue;
    }

    playerModel.players.push_back(potentialPlayer);
  }

  // --------------------------Drawing---------------------------------
  //
  for (const auto& p : playerModel.players) {
    Vector2<> globalPos = theRobotPose * p.relPosOnField;
    CIRCLE("module:PlayerLocator:players",
           globalPos.x,
           globalPos.y,
           150,
           20,
           Drawings::ps_solid,
           ColorClasses::blue,
           Drawings::bs_null,
           ColorRGBA());
  }
  for (const auto& p : playerModel.potentialPlayers) {
    Vector2<> globalPos = theRobotPose * p.relPosOnField;
    CIRCLE("module:PlayerLocator:potentialPlayers",
           globalPos.x,
           globalPos.y,
           150,
           20,
           Drawings::ps_solid,
           ColorClasses::green,
           Drawings::bs_null,
           ColorRGBA());
  }
  // Logging
  if (enableLogging) {
    std::ofstream myfile;
    myfile.open("/home/nao/logs/PlayerPercept.log", std::ios::app);
    myfile << theFrameInfo.time;
    myfile << ", " << thePlayerPercept.players.size();
    for (auto& perceptedPlayer : thePlayerPercept.players) {
      myfile << ", " << perceptedPlayer.centerOnField.x << ", " << perceptedPlayer.centerOnField.y;
    }
    myfile << std::endl;
    myfile.close();

    myfile.open("/home/nao/logs/PlayerModel.log", std::ios::app);
    myfile << theFrameInfo.time;
    myfile << ", " << playerModel.players.size() << ", " << playerModel.potentialPlayers.size();
    for (auto& player : playerModel.players) {
      myfile << ", " << player.relPosOnField.x << ", " << player.relPosOnField.y;
    }
    myfile << std::endl;
    myfile.close();
  }

  // std::cout << "numPlayers: " << playerModel.players.size() << std::endl;

  /*auto minDistance = INFINITY;
  for(auto& player : playerModel.players) {
      std::cout << " PLAYER: " << player.relPosOnField.abs() << std::endl;
      minDistance = std::min(minDistance, player.relPosOnField.abs());
  }
  std::cout << "LOCATOR: " << thePlayerPercept.players.size() << " " << playerModel.potentialPlayers.size() << " " <<
  playerModel.players.size() << " " << minDistance << std::endl;*/
}

bool PlayerLocator::isPlayerOutsideOfField(const PlayerModel::Player& player) {
  Vector2<> globalPlayerPos = theRobotPose * player.relPosOnField;

  // xAxis check
  if ((globalPlayerPos.x > (theFieldDimensions.xPosOpponentGroundline + playerOutsideFieldOffset)) ||
      (globalPlayerPos.x < (-1.f * theFieldDimensions.xPosOpponentGroundline - playerOutsideFieldOffset))) {
    return true;
  }

  // yAxis check
  if ((globalPlayerPos.y > (theFieldDimensions.yPosLeftSideline + playerOutsideFieldOffset)) ||
      (globalPlayerPos.y < (-1.f * theFieldDimensions.yPosLeftSideline - playerOutsideFieldOffset))) {
    return true;
  }

  return false;
}
