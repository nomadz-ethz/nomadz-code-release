#pragma once

#include "behaviortree_cpp/action_node.h"

namespace nomadz_behavior {

  class SpecialAction : public BT::StatefulActionNode {
  public:
    SpecialAction(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("SpecialAction");
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class Penalty : public BT::StatefulActionNode {
  public:
    Penalty(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("Penalty");
    }
    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class Recovery : public BT::StatefulActionNode {
  public:
    Recovery(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("Recovery");
    }
    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
    uint8_t counter_back_ = 0;
  };

  class Relax : public BT::StatefulActionNode {
  public:
    Relax(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("Relax");
    }
    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class SaveShot : public BT::StatefulActionNode {
  public:
    SaveShot(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("SaveShot");
    }
    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };
} // namespace nomadz_behavior
