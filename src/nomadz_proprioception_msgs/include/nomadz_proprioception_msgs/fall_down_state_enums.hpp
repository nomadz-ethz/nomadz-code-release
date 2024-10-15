#pragma once

namespace nomadz_proprioception_msgs {
  enum class FallDownState { UNDEFINED, UPRIGHT, STAGGERING, FALLING, ON_GROUND };
  enum class FallDirection { NONE, FRONT, BACK, LEFT, RIGHT };
} // namespace nomadz_proprioception_msgs
