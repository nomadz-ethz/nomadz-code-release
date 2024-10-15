#pragma once

#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include "nomadz_behavior/actions.hpp"
#include "nomadz_behavior/checks.hpp"
#include "nomadz_behavior/ros_interface.hpp"
#include "nomadz_behavior/helper/target_pose_generator.hpp"
#include "nomadz_configuration/game_settings.hpp"

#include "nomadz_motion_control_msgs/msg/motion_request.hpp"

namespace nomadz_behavior {
  class Behavior {
  private:
    constexpr static float BT_LOOP_FREQUENCY = 0.5F;

  public:
    Behavior();

    void initConfigurations();
    void initHelperModules();
    void initRosInterface();

    void run();

    nomadz_configuration::GameSettings game_settings_;

  private:
    void saveTreeModel();

    BT::Blackboard::Ptr global_blackboard_;
    BT::BehaviorTreeFactory factory_;

    std::shared_ptr<RosInterfaceNode> ros_interface_node_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    std::shared_ptr<TargetPoseGenerator> target_pose_generator_;

    CheckNodes check_nodes_;
    ActionNodes action_nodes_;

    std::filesystem::path behavior_cfg_folder_;

    YAML::Node behavior_cfg_;
    BT::Tree motion_behavior_tree_;
    BT::Tree head_motion_behavior_tree_;
  };

} // namespace nomadz_behavior
