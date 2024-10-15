#pragma once

#include <behaviortree_cpp/basic_types.h>

#include "nomadz_core/geometry/twist.hpp"
#include "nomadz_core/geometry/pose.hpp"

namespace BT {
  template <> inline nomadz_core::Twist2D convertFromString(StringView str) {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ' ');
    if (parts.size() != 3) {
      throw RuntimeError("invalid input)");
    } else {
      nomadz_core::Twist2D output;
      output.x = convertFromString<float>(parts[0]);
      output.y = convertFromString<float>(parts[1]);
      output.theta = convertFromString<float>(parts[2]);
      return output;
    }
  }

  template <> inline nomadz_core::Pose2D convertFromString(StringView str) {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ' ');
    if (parts.size() != 3) {
      throw RuntimeError("invalid input)");
    } else {
      nomadz_core::Pose2D output;
      output.x = convertFromString<float>(parts[0]);
      output.y = convertFromString<float>(parts[1]);
      output.theta = convertFromString<float>(parts[2]);
      return output;
    }
  }
} // end namespace BT
