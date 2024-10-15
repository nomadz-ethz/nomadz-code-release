#pragma once

#include <array>
#include <chrono>

#include "nomadz_definitions/limbs.hpp"

#include "nomadz_proprioception/proprioception_enums.hpp"

namespace nomadz_proprioception {

  using FsrArray = std::array<std::array<float, Fsr::NUM_FSRS>, nomadz_definitions::side::NUM_SIDES>;
  using Timestamp = std::chrono::steady_clock::time_point;

} // namespace nomadz_proprioception
