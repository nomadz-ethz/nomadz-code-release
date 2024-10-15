#pragma once

namespace nomadz_motion_control_msgs {
  enum class SpecialActionType {
    PLAY_DEAD,
    SIT_DOWN,
    GO_UP,
    STAND,
    STAND_HIGH,
    FALL_PROTECTION_BACK,
    FALL_PROTECTION_FRONT,
    FALL_PROTECTION_SIDE,
    GET_UP_BACK_NAO_22,
    GET_UP_FRONT_NAO_22,
    SIT_DOWN_GOALKEEPER,
    NUM_SPECIAL_ACTIONS
  };

} // namespace nomadz_motion_control_msgs
