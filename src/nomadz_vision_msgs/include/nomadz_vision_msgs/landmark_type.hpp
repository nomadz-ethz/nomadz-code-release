#pragma once

#include <cstdint>

namespace nomadz_vision_msgs {
  enum class LandmarkType : uint8_t { PENALTY_MARK = 0, CIRCLE };

  inline bool operator==(uint8_t lhs, const LandmarkType& rhs) {
    return lhs == static_cast<uint8_t>(rhs);
  }
} // namespace nomadz_vision_msgs
