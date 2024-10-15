#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

#include "nomadz_configuration/field_dimensions.hpp"
#include "nomadz_behavior/data_types/motion.hpp"

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace nomadz_behavior {

  class WalkAtSpeed : public BT::SyncActionNode {
  public:
    WalkAtSpeed(const std::string& name, const BT::NodeConfiguration& config)
        : SyncActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("WalkAtSpeed");
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class WalkToTarget : public BT::StatefulActionNode {
  public:
    WalkToTarget(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("WalkToTarget");
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class Alignment : public BT::StatefulActionNode {
  public:
    Alignment(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("Alignment");
    }
    const nomadz_configuration::Dims dims_;
    const Eigen::Vector2f opp_goal_center_ = Eigen::Vector2f{dims_.x_pos_opponent_ground_line, 0.0};
    const Eigen::Vector2f opp_penalty_mark_ = Eigen::Vector2f{dims_.x_pos_opponent_penalty_mark, 0.0};
    const YAML::Node behavior_cfg = config().blackboard->rootBlackboard()->get<YAML::Node>("behavior_cfg");
    const float angular_tolerance = behavior_cfg["has_ball_distance"].as<float>();
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    // helper function
    std::tuple<MotionRequest, float, float> calculateMotionRequest(std::string set_play);

  private:
    BT::Blackboard::Ptr blackboard_;
    std::string set_play_;
  };

  class FootAlignment : public BT::StatefulActionNode {
  public:
    FootAlignment(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("FootAlignment");
    }
    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

} // namespace nomadz_behavior
