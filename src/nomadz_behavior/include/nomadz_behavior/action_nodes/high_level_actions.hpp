#pragma once

#include "behaviortree_cpp/action_node.h"

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "nomadz_behavior/data_types/motion.hpp"
#include "nomadz_configuration/field_dimensions.hpp"
#include "nomadz_core/geometry/pose.hpp"

namespace nomadz_behavior {
  class ApplySkill : public BT::StatefulActionNode {
  public:
    ApplySkill(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("ApplySkill");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
    const YAML::Node behavior_cfg = blackboard_->get<YAML::Node>("behavior_cfg");
    const float dribble_possible_distance = behavior_cfg["dribble_possible_distance"].as<float>();
    const float dribble_kick_power = behavior_cfg["dribble_kick_power"].as<float>();
    const nomadz_configuration::Dims dims_;
  };

  class Engage : public BT::SyncActionNode {
  public:
    Engage(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config) {
      setRegistrationID("Engage");
    }
    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;
  };

  class Kick : public BT::StatefulActionNode {
  public:
    Kick(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("Kick");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
    const YAML::Node behavior_cfg_ = blackboard_->get<YAML::Node>("behavior_cfg");
    const float penalty_kick_power_ = behavior_cfg_["penalty_kick_power"].as<float>();
    const nomadz_configuration::Dims dims_;
  };
} // namespace nomadz_behavior
