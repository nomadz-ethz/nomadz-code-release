#pragma once

#include "behaviortree_cpp/action_node.h"

namespace nomadz_behavior {
  class GetTarget : public BT::SyncActionNode {
  public:
    GetTarget(const std::string& name, const BT::NodeConfiguration& config)
        : SyncActionNode(name, config), blackboard_(config.blackboard->rootBlackboard()) {
      setRegistrationID("GetTarget");
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    BT::Blackboard::Ptr blackboard_;
  };
} // namespace nomadz_behavior
