#include "nomadz_behavior/helper/bt_enums.hpp"

#include "nomadz_behavior/data_types/game_status.hpp"
#include "nomadz_motion_control_msgs/special_action_request_enums.hpp"

namespace nomadz_behavior {
  void registerBtEnums(BT::BehaviorTreeFactory& factory) {
    factory.registerScriptingEnums<TargetType>();
    factory.registerScriptingEnums<GameStatus::GamePhase>();
    factory.registerScriptingEnums<GameStatus::GameState>();
    factory.registerScriptingEnums<nomadz_motion_control_msgs::SpecialActionType>();
  }
} // namespace nomadz_behavior
