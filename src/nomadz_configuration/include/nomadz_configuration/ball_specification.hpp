#pragma once

namespace nomadz_configuration {

  /**
   * @brief Ball specification
   *
   * The default values are based on the RoboCup SPL 2024 rules.
   */
  struct BallSpecification {
    float ball_radius = 0.05F;    // m
    float ball_friction = -0.30F; // m/(s*s)
  };

} // namespace nomadz_configuration
