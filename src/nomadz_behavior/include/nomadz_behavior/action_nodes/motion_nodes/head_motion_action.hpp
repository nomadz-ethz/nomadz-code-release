#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace nomadz_behavior {
  class LookAtBall : public BT::StatefulActionNode {
  public:
    LookAtBall(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("LookAtBall");
    }
    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

  class LookAtDirection : public BT::StatefulActionNode {
  public:
    LookAtDirection(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("LookAtDirection");
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };

} // namespace nomadz_behavior
