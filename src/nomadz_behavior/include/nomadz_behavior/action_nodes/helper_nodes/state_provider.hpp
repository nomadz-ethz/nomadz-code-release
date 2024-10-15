#pragma once

#include <map>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "nomadz_behavior/data_types/game_status.hpp"

namespace nomadz_behavior {
  class GameStatusProvider : public BT::SyncActionNode {
  private:
    inline static const std::map<GameStatus::GamePhase, std::string> GAME_PHASE_MAP = {
      {GameStatus::GamePhase::NORMAL, "NORMAL"},
      {GameStatus::GamePhase::PENALTYSHOOT, "PENALTYSHOOT"},
      {GameStatus::GamePhase::OVERTIME, "OVERTIME"},
      {GameStatus::GamePhase::TIMEOUT, "TIMEOUT"}};

    inline static const std::map<GameStatus::GameState, std::string> GAME_STATE_MAP = {
      {GameStatus::GameState::INITIAL, "INITIAL"},
      {GameStatus::GameState::READY, "READY"},
      {GameStatus::GameState::SET, "SET"},
      {GameStatus::GameState::PLAYING, "PLAYING"},
      {GameStatus::GameState::FINISHED, "FINISHED"},
      {GameStatus::GameState::STANDBY, "STANDBY"}};

    inline static const std::map<GameStatus::SetPlay, std::string> SET_PLAY_MAP = {
      {GameStatus::SetPlay::NONE, "NONE"},
      {GameStatus::SetPlay::GOAL_KICK, "GOAL_KICK"},
      {GameStatus::SetPlay::PUSHING_FREE_KICK, "PUSHING_FREE_KICK"},
      {GameStatus::SetPlay::CORNER_KICK, "CORNER_KICK"},
      {GameStatus::SetPlay::KICK_IN, "KICK_IN"},
      {GameStatus::SetPlay::PENALTY_KICK, "PENALTY_KICK"}};

  public:
    GameStatusProvider(const std::string& name, const BT::NodeConfiguration& config)
        : SyncActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("GameStatusProvider");
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class RoleProvider : public BT::SyncActionNode {
  public:
    RoleProvider(const std::string& name, const BT::NodeConfiguration& config)
        : SyncActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("RoleProvider");
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class LogUndefinedState : public BT::SyncActionNode {
  public:
    LogUndefinedState(const std::string& name, const BT::NodeConfiguration& config)
        : SyncActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("LogUndefinedState");
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };
} // namespace nomadz_behavior
