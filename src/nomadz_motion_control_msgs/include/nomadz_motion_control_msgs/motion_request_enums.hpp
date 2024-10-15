#pragma once

#include "nomadz_motion_control_msgs/special_action_request_enums.hpp"
#include "nomadz_motion_control_msgs/head_motion_request_enums.hpp"
#include "nomadz_motion_control_msgs/walk_request_enums.hpp"
#include "nomadz_motion_control_msgs/kick_request_enums.hpp"

namespace nomadz_motion_control_msgs {
  enum class MotionType { SPECIAL_ACTION, WALK, KICK };
} // namespace nomadz_motion_control_msgs
