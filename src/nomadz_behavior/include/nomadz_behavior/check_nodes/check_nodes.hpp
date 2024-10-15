#pragma once

#include "behaviortree_cpp/condition_node.h"

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "nomadz_configuration/field_dimensions.hpp"
#include "nomadz_configuration/game_settings.hpp"

namespace nomadz_behavior {
  class Penalized : public BT::ConditionNode {
  public:
    Penalized(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("Penalized");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class Standing : public BT::ConditionNode {
  public:
    Standing(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("Standing");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class Localized : public BT::ConditionNode {
  public:
    Localized(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("Localized");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class Lost : public BT::ConditionNode {
  public:
    Lost(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("Lost");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class BallFound : public BT::ConditionNode {
  public:
    BallFound(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("BallFound");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class BallLost : public BT::ConditionNode {
  public:
    BallLost(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("BallLost");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class HasBallLock : public BT::ConditionNode {
  public:
    HasBallLock(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("HasBallLock");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class BallFree : public BT::ConditionNode {
  public:
    BallFree(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("BallFree");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class HasBall : public BT::ConditionNode {
  public:
    HasBall(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("HasBall");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
    const YAML::Node behavior_cfg = blackboard_->get<YAML::Node>("behavior_cfg");
    const float has_ball_distance = behavior_cfg["has_ball_distance"].as<float>();
  };

  class IsHighestActivePlayer : public BT::ConditionNode {
  public:
    IsHighestActivePlayer(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("IsHighestActivePlayer");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
    const int player_id_ = blackboard_->get<nomadz_configuration::GameSettings>("game_settings").player_id;
  };

  class IsAligned : public BT::ConditionNode {
  public:
    IsAligned(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("IsAligned");
    }
    const YAML::Node behavior_cfg = config().blackboard->rootBlackboard()->get<YAML::Node>("behavior_cfg");
    const float angular_tolerance = behavior_cfg["is_aligned_angular_tolerance"].as<float>();
    const nomadz_configuration::FieldLines field_lines =
      config().blackboard->rootBlackboard()->get<nomadz_configuration::FieldLines>("field_lines");
    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class CanSaveShot : public BT::ConditionNode {
  public:
    const nomadz_configuration::Dims field_dims;
    const float distance_keeper_penalty_mark =
      field_dims.x_pos_opponent_ground_line - field_dims.x_pos_opponent_penalty_mark;
    CanSaveShot(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("CanSaveShot");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class IsInsideArea : public BT::ConditionNode {
  public:
    const nomadz_configuration::Corners corners;
    // orientation as seen from the own goalkeeper when looking at the field and the opponent goal
    const Eigen::Vector2f left_penalty_area_corner = corners.L_CORNER_180.at(8);
    const Eigen::Vector2f right_penalty_area_corner = corners.L_CORNER_90.at(8);
    IsInsideArea(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("IsInsideArea");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

} // namespace nomadz_behavior
