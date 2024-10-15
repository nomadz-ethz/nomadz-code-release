#pragma once

#include "nomadz_communication_msgs/robocup_game_control_data_enums.hpp"

namespace nomadz_behavior {
  struct GameStatus {
    using GamePhase = nomadz_communication_msgs::GamePhase;
    using GameState = nomadz_communication_msgs::GameState;
    using SetPlay = nomadz_communication_msgs::SetPlay;
    GamePhase game_phase;
    GameState game_state;
    SetPlay set_play;
    bool is_first_half;
    bool is_kicking_team;
  };
} // namespace nomadz_behavior
