/**
 * @file PlayerModel.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:afabisch@tzi.de>Alexander Fabisch</a>
 */

#pragma once

#include <vector>

#include "Core/Math/Matrix2x2.h"
#include "Core/Math/Random.h"
#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/FieldWrapper.h"

/**
 * @class PlayerModel
 * This representation contains all percepted and smoothed players in relative
 * field coordinates.
 */
#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/player.hpp"
#endif
STREAMABLE_DECLARE(Player)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/player_model.hpp"
#endif
STREAMABLE_DECLARE(PlayerModel)

STREAMABLE_ROS(PlayerModel, {
public:
  /**
   * @class Player
   * A player on the field.
   */
  STREAMABLE_ROS(Player, {
    public : Player(
      const Vector2<>& relPosOnField, bool opponent, bool standing, const Matrix2x2<>& covariance, unsigned timeStamp),
    FIELD_WRAPPER_DEFAULT(Vector2<>,
                          nomadz_msgs::msg::Player::rel_pos_on_field,
                          relPosOnField), /**< Relative position of the player on the field. */
    FIELD_WRAPPER_DEFAULT(bool, nomadz_msgs::msg::Player::opponent, opponent), /**< Opponent or own team? */
    FIELD_WRAPPER_DEFAULT(bool, nomadz_msgs::msg::Player::standing, standing), /**< Is this player standing or not */
    FIELD_WRAPPER_DEFAULT(Matrix2x2<>, nomadz_msgs::msg::Player::covariance, covariance), /**< covariance of a seen player */
    FIELD_WRAPPER_DEFAULT(
      float, nomadz_msgs::msg::Player::min_variance, minVariance), /**< Minimum variance of a seen player */
    FIELD_WRAPPER_DEFAULT(unsigned, nomadz_msgs::msg::Player::time_stamp, timeStamp), /**< Timestamp of the last update. */
    FIELD_WRAPPER_DEFAULT(unsigned, nomadz_msgs::msg::Player::first_seen, firstSeen), /**< Timestamp when first seen. */
    FIELD_WRAPPER_DEFAULT(
      bool, nomadz_msgs::msg::Player::detected, detected), /**< Has this player been detected in the current frame */
  });

  typedef std::vector<Player>::const_iterator RCIt;
  typedef std::vector<Player>::iterator RIt;

  void draw() const, FIELD_WRAPPER_DEFAULT(std::vector<Player>, nomadz_msgs::msg::PlayerModel::players, players),
    FIELD_WRAPPER_DEFAULT(std::vector<Player>, nomadz_msgs::msg::PlayerModel::potential_players, potentialPlayers),
});

/**
 * @class GroundTruthPlayerModel
 * The class contains the true PlayerModel.
 */
STREAMABLE_ALIAS(GroundTruthPlayerModel, PlayerModel, {});

/**
 * @class PlayerModelCompressed
 * This class contains a compressed PlayerModel for team communication.
 */
STREAMABLE_DECLARE_LEGACY(PlayerCompressed)
STREAMABLE_DECLARE_LEGACY(PlayerModelCompressed)
STREAMABLE_LEGACY(PlayerModelCompressed, {
public:
  STREAMABLE_LEGACY(PlayerCompressed, {
  public:
    PlayerCompressed(const PlayerModel::Player& player);
    operator PlayerModel::Player() const,

      // 36 Bytes -> 22 Bytes
      (Vector2<short>)relPosOnField, (bool)opponent, (bool)standing, (float)covXX, (float)covYY,
      (float)covXY, // == covYX
      (unsigned)timeStamp,
  });

  using Player = PlayerCompressed;

  PlayerModelCompressed(const PlayerModel& playerModel, unsigned int maxNumberOfPlayers);
  operator PlayerModel() const,

    (std::vector<Player>)players,
});
