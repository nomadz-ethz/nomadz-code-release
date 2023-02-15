/**
 * @file PlayerPercept.h
 *
 * Representation of players detected in the image
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <limits>
#include "Core/Streams/AutoStreamable.h"
#include "Core/Math/Vector2.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/percept_player.hpp"
#endif
STREAMABLE_DECLARE(PerceptPlayer)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/player_percept.hpp"
#endif
STREAMABLE_DECLARE(PlayerPercept)

STREAMABLE_ROS(PlayerPercept, {
public:
  STREAMABLE_ROS(
    PerceptPlayer,
    {
      ,
      FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PerceptPlayer::x1, x1), // left border in the image

      FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PerceptPlayer::y1, y1), // top border in the image

      FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PerceptPlayer::x2, x2), // right border in the image

      FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PerceptPlayer::y2, y2), // bottom border in the image

      FIELD_WRAPPER_DEFAULT(Vector2<>,
                            nomadz_msgs::msg::PerceptPlayer::center_on_field,
                            centerOnField), // player center in relative to the robot in field coordinates

      FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::PerceptPlayer::radius, radius),
      FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PerceptPlayer::jersey_y0, jerseyY0), // top of possible jersey in image

      FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PerceptPlayer::jersey_y1, jerseyY1), // bottom of possible jersey in image

      FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PerceptPlayer::detected_jersey, detectedJersey),
      FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PerceptPlayer::opponent, opponent),
      FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PerceptPlayer::standing, standing),
    });

  using Player = PerceptPlayer;

  /** Draws the players */
  void draw() const,
    FIELD_WRAPPER_DEFAULT(
      std::vector<PerceptPlayer>, nomadz_msgs::msg::PlayerPercept::players, players), /**< Vector of all players */
});
