#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace nomadz_behavior {

  class PrintBlackboardNode : public BT::SyncActionNode {
  public:
    PrintBlackboardNode(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config) {
      setRegistrationID("PrintBlackboardNode");
    }

    static BT::PortsList providedPorts() { return BT::PortsList{}; };
    BT::NodeStatus tick() override;
  };

} // namespace nomadz_behavior
