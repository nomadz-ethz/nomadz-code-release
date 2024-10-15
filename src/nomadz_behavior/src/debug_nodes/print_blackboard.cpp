#include "nomadz_behavior/debug_nodes/print_blackboard.hpp"

#include <spdlog/spdlog.h>

#include <iostream>

namespace nomadz_behavior {
  BT::NodeStatus PrintBlackboardNode::tick() {
    // Access the blackboard
    auto blackboard = config().blackboard; //->rootBlackboard();
    spdlog::info("Blackboard contents:");

    // Print all key in the blackboard
    for (const auto& key : blackboard->getKeys()) {
      spdlog::info(key);
    }
    return BT::NodeStatus::SUCCESS;
  }
} // namespace nomadz_behavior
