#pragma once

#include <cstdint>
#include <vector>

namespace nomadz_communication {
  static constexpr int NUM_PLAYERS_TO_SEND = 0;

  struct EgoStatusCompact {
    bool whistle_detected;
    bool has_fallen;

    bool want_ball;
    bool has_ball_lock;
    float ball_score;
  };

  // For int8_t, the range is [-128, 127]. We are not using -128 for symmetry reason.
  // For x position -127 is used to represent -5.5m and 127 is used to represent 5.5m
  // For y position -127 is used to represent -4m and 127 is used to represent 4m
  // For theta -127 is used to represent -pi and 127 is used to represent pi
  struct BallModelCompact {
    int8_t x; // [-127=-1, 127=1]
    int8_t y; // [-127, 127]
    bool valid;
  };

  struct RobotPoseCompact {
    int8_t x;     // [-127 , 127]
    int8_t y;     // [-127 , 127]
    int8_t theta; // [-127 = -pi, 127 = pi]
    bool valid;
  };

  struct PlayerModelCompact {
    int8_t x;
    int8_t y;
    int8_t theta;
    int8_t role;
  };

  struct TeamCommData {
    uint8_t player_id;

    EgoStatusCompact ego_status{};
    BallModelCompact ball_model{};
    RobotPoseCompact robot_pose{};
    // std::vector<PlayerModelCompact> player_models;
  };

} // namespace nomadz_communication
