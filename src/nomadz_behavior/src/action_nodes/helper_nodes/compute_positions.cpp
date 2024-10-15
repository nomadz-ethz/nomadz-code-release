#include "nomadz_behavior/action_nodes/helper_nodes/compute_positions.hpp"

#include <spdlog/spdlog.h>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_behavior/helper/bt_enums.hpp"
#include "nomadz_behavior/helper/target_pose_generator.hpp"

using Pose2D = nomadz_core::Pose2D;

namespace nomadz_behavior {
  BT::PortsList GetTarget::providedPorts() {
    BT::PortsList ports_list;
    ports_list.emplace(BT::InputPort<std::string>("target_type"));
    ports_list.emplace(BT::OutputPort<Pose2D>("target"));
    return ports_list;
  }

  BT::NodeStatus GetTarget::tick() {
    BT::Expected<TargetType> target_type = getInput<TargetType>("target_type");

    spdlog::info("TICK GetTarget: {0:d}", static_cast<uint8_t>(target_type.value()));
    Pose2D target_pose{};
    if (blackboard_->get<std::shared_ptr<TargetPoseGenerator>>("target_pose_generator")
          ->getTargetPose(target_type.value(), target_pose)) {
      setOutput("target", target_pose);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
} // namespace nomadz_behavior
