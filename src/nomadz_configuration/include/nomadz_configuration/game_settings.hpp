#pragma once

#include <string>

namespace nomadz_configuration {
  struct GameSettings {
    int team_id = 33;
    int player_id = 1;
    std::string player_role = "Player";
    std::string team_color = "blue";
    std::string keeper_color = "orange";
    std::string location = "default";
  };
} // namespace nomadz_configuration
