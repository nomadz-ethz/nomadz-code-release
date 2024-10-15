#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace nomadz_behavior {
  /// Type of target
  enum class TargetType {
    GAME_READY = 0,
    SEARCH = 1,
    COVER = 2,
    OBSERVATION = 3, /// when ball is not free go to pose between you and ball 1m away from ball
    ENGAGE = 4,      /// target is ball
    CENTER_GOAL = 5,
    DETECT_SHOT = 6,
    PENALTY_KICK = 7
  };

  void registerBtEnums(BT::BehaviorTreeFactory& factory);
} // namespace nomadz_behavior
