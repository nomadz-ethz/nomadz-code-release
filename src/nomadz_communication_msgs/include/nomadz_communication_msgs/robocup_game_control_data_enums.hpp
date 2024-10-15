#pragma once

#include <cstdint>

namespace nomadz_communication_msgs {
  enum class CompetitionPhase : uint8_t {
    ROUND_ROBIN = 0,
    PLAY_OFF = 1

  };
  enum class CompetitionType : uint8_t { NORMAL = 0, SHARED_AUTONOMY = 1 };

  enum class GamePhase : uint8_t { NORMAL = 0, PENALTYSHOOT = 1, OVERTIME = 2, TIMEOUT = 3 };

  enum class GameState : uint8_t { INITIAL = 0, READY = 1, SET = 2, PLAYING = 3, FINISHED = 4, STANDBY = 5 };

  enum class SetPlay : uint8_t {
    NONE = 0,
    GOAL_KICK = 1,
    PUSHING_FREE_KICK = 2,
    CORNER_KICK = 3,
    KICK_IN = 4,
    PENALTY_KICK = 5
  };

  enum class Penalty : uint8_t {
    NONE = 0,
    SPL_ILLEGAL_BALL_CONTACT = 1,
    SPL_PLAYER_PUSHING = 2,
    SPL_ILLEGAL_MOTION_IN_SET = 3,
    SPL_INACTIVE_PLAYER = 4,
    SPL_ILLEGAL_POSITION = 5,
    SPL_LEAVING_THE_FIELD = 6,
    SPL_REQUEST_FOR_PICKUP = 7,
    SPL_LOCAL_GAME_STUCK = 8,
    SPL_ILLEGAL_POSITION_IN_SET = 9,
    SPL_PLAYER_STANCE = 10,
    SPL_ILLEGAL_MOTION_IN_STANDBY = 11,
    SUBSTITUTE = 14,
    MANUAL = 15
  };

  enum class TeamJerseyColor : uint8_t {
    BLUE = 0,   // blue, cyan
    RED = 1,    // red, magenta, pink
    YELLOW = 2, // yellow
    BLACK = 3,  // black, dark gray
    WHITE = 4,  // white
    GREEN = 5,  // green
    ORANGE = 6, // orange
    PURPLE = 7, // purple, violet
    BROWN = 8,  // brown
    GRAY = 9,   // lighter gray
  };
} // namespace nomadz_communication_msgs
