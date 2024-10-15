#pragma once

#include <cstdint>

namespace nomadz_proprioception {
  enum Fsr { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, NUM_FSRS };
  enum class PushDirection : std::uint8_t { FORWARD, BACKWARD, LEFT, RIGHT, NONE };
  enum class Camera : std::uint8_t { UPPER_CAMERA, LOWER_CAMERA };

} // namespace nomadz_proprioception
