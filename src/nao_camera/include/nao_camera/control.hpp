#pragma once

#include <linux/videodev2.h>

#include <cstdint>
#include <string>
#include <map>

namespace nao_camera {

  /// Type of camera control
  enum class ControlType : std::uint8_t {
    INT = 1,
    BOOL = 2,
    MENU = 3,
    BUTTON = 4,
    INT64 = 5,
    CTRL_CLASS = 6,
    STRING = 7,
    BITMASK = 8
  };

  struct Control {
    /// Identifies the control, set by the application
    unsigned id;

    /// Human readable name
    std::string name;

    /// Type of control
    ControlType type;

    /// Minimum value, inclusive
    int minimum;

    /// Maximum value, inclusive
    int maximum;

    /// The default value of of an integer, boolean, bitmask, menu or integer menu control
    int default_value;

    /// Menu item names by index. Empty if this is not a menu control
    std::map<int, std::string> menu_items;

    /// Whether the control is set to inactive, e.g. when it is automatically controlled
    bool inactive;
  };

} // namespace nao_camera
