/**
 * @file PlayerModel.cpp
 *
 * Debug drawings for the PlayerModel.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:afabisch@tzi.de>Alexander Fabisch</a>
 */

#include "PlayerModel.h"
#include "Core/System/BHAssert.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Math/Covariance.h"

PlayerModel::Player::Player(
  const Vector2<>& relPosOnField, bool teamRed, bool standing, const Matrix2x2<>& covariance, unsigned timeStamp)
    : Player() {
  this->relPosOnField = relPosOnField;
  this->opponent = opponent;
  this->standing = standing;
  this->covariance = covariance;
  this->timeStamp = timeStamp;
}

void PlayerModel::draw() const {
  DECLARE_DEBUG_DRAWING("representation:PlayerModel:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:PlayerModel:covariance", "drawingOnField");

  COMPLEX_DRAWING("representation:PlayerModel:covariance", {
    for (RCIt r(players.begin()); r != players.end(); r++) {
      float s1 = 0.0, s2 = 0.0, theta = 0.0;
      Covariance::errorEllipse(r->covariance, s1, s2, theta);
      ELLIPSE("representation:PlayerModel:covariance",
              r->relPosOnField,
              sqrt(3.0f) * s1,
              sqrt(3.0f) * s2,
              theta,
              10,
              Drawings::ps_solid,
              ColorRGBA(100, 100, 255, 100),
              Drawings::bs_solid,
              ColorRGBA(100, 100, 255, 100));
      ELLIPSE("representation:PlayerModel:covariance",
              r->relPosOnField,
              sqrt(2.0f) * s1,
              sqrt(2.0f) * s2,
              theta,
              10,
              Drawings::ps_solid,
              ColorRGBA(150, 150, 100, 100),
              Drawings::bs_solid,
              ColorRGBA(150, 150, 100, 100));
      ELLIPSE("representation:PlayerModel:covariance",
              r->relPosOnField,
              s1,
              s2,
              theta,
              10,
              Drawings::ps_solid,
              ColorRGBA(255, 100, 100, 100),
              Drawings::bs_solid,
              ColorRGBA(255, 100, 100, 100));
    }
  });

  COMPLEX_DRAWING("representation:PlayerModel:Field", {
    for (RCIt r(players.begin()); r != players.end(); r++) {
      ColorClasses::Color color(r->opponent ? ColorClasses::red : ColorClasses::blue);
      CROSS("representation:PlayerModel:Field", r->relPosOnField.x, r->relPosOnField.y, 50, 20, Drawings::ps_solid, color);
    }
  });
}

PlayerModelCompressed::PlayerCompressed::PlayerCompressed(const PlayerModel::Player& player)
    : relPosOnField(player.relPosOnField), opponent(player.opponent), standing(player.standing),
      covXX(player.covariance[0][0]), covYY(player.covariance[1][1]), covXY(player.covariance[0][1]),
      timeStamp(player.timeStamp) {}

PlayerModelCompressed::Player::operator PlayerModel::Player() const {
  return PlayerModel::Player(
    Vector2<>(relPosOnField), opponent, standing, Matrix2x2<>(covXX, covXY, covXY, covYY), timeStamp);
}

PlayerModelCompressed::PlayerModelCompressed(const PlayerModel& playerModel, unsigned int maxNumberOfPlayers) {
  unsigned int offset = 0;
  const unsigned int numOfInputPlayers = playerModel.players.size();
  unsigned int numOfUsedPlayers = numOfInputPlayers;
  if (numOfUsedPlayers > maxNumberOfPlayers) {
    numOfUsedPlayers = maxNumberOfPlayers;
    offset = static_cast<unsigned int>(random(static_cast<int>(numOfInputPlayers)));
  }
  players.reserve(numOfUsedPlayers);
  for (size_t i = 0; i < numOfUsedPlayers; i++) {
    players.push_back(Player(playerModel.players[(offset + i) % numOfInputPlayers]));
  }
}

PlayerModelCompressed::operator PlayerModel() const {
  PlayerModel playerModel;
  playerModel.players.reserve(players.size());
  for (size_t i = 0; i < players.size(); i++) {
    playerModel.players.push_back(players[i]);
  }
  return playerModel;
}
