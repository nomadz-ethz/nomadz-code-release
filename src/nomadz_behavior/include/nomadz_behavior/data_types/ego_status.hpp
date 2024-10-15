#pragma once

namespace nomadz_behavior {
  struct EgoStatus {
    bool whistle_detected;
    bool is_penalized;
    bool has_fallen;

    bool want_ball;
    bool has_ball_lock;
    float ball_score;
  };

} // namespace nomadz_behavior
