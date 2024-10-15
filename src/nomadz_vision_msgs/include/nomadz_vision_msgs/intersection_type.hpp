#pragma once

#include <cstdint>

namespace nomadz_vision_msgs {
  enum class IntersectionType : uint8_t { L = 0, T, X };

  inline bool operator==(uint8_t lhs, const IntersectionType& rhs) {
    return lhs == static_cast<uint8_t>(rhs);
  }
} // namespace nomadz_vision_msgs
