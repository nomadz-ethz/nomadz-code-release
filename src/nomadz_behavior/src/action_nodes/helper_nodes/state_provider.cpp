
#include "nomadz_behavior/action_nodes/helper_nodes/state_provider.hpp"
#include "nomadz_configuration/game_settings.hpp"

#include <spdlog/spdlog.h>

namespace nomadz_behavior {
  BT::PortsList GameStatusProvider::providedPorts() {
    BT::PortsList ports_list;
    ports_list.emplace(BT::OutputPort<std::string>("game_phase"));
    ports_list.emplace(BT::OutputPort<std::string>("game_state"));
    ports_list.emplace(BT::OutputPort<std::string>("set_play"));
    ports_list.emplace(BT::OutputPort<bool>("is_kicking_team"));

    return ports_list;
  }

  BT::NodeStatus GameStatusProvider::tick() {
    spdlog::info("TICK GameStatusProvider.");
    GameStatus game_status = blackboard_->get<GameStatus>("game_status");
    setOutput("game_phase", GAME_PHASE_MAP.at(game_status.game_phase));
    setOutput("game_state", GAME_STATE_MAP.at(game_status.game_state));
    setOutput("set_play", SET_PLAY_MAP.at(game_status.set_play));
    setOutput("is_kicking_team", game_status.is_kicking_team);

    return BT::NodeStatus::SUCCESS;
  }

  BT::PortsList RoleProvider::providedPorts() {
    BT::PortsList ports_list;
    ports_list.emplace(BT::OutputPort<std::string>("player_role"));

    return ports_list;
  }

  BT::NodeStatus RoleProvider::tick() {
    spdlog::info("TICK RoleProvider.");
    auto game_settings = blackboard_->get<nomadz_configuration::GameSettings>("game_settings");
    setOutput("player_role", game_settings.player_role);

    return BT::NodeStatus::SUCCESS;
  }

  BT::PortsList LogUndefinedState::providedPorts() {
    BT::PortsList ports_list;
    ports_list.emplace(BT::InputPort<std::string>("state_name"));
    ports_list.emplace(BT::InputPort<std::string>("requested_state"));

    return ports_list;
  }

  BT::NodeStatus LogUndefinedState::tick() {
    BT::Expected<std::string> state_name = getInput<std::string>("state_name");
    BT::Expected<std::string> requested_state = getInput<std::string>("requested_state");

    spdlog::warn("TICK LogUndefinedState: " + requested_state.value() + " not valid for state " + state_name.value());

    return BT::NodeStatus::SUCCESS;
  }

} // namespace nomadz_behavior
