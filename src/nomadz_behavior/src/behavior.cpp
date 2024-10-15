#include "nomadz_behavior/behavior.hpp"

#include <fstream>
#include <spdlog/spdlog.h>
#include <rclcpp/logging.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/xml_parsing.h>
#ifndef TARGET_ROBOT
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#endif

#include "nomadz_configuration/io.hpp"
#include "nomadz_behavior/helper/bt_enums.hpp"
#include "nomadz_configuration/field_dimensions.hpp"

namespace fs = std::filesystem;

namespace nomadz_behavior {
  Behavior::Behavior() : check_nodes_(factory_), action_nodes_(factory_) {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nomadz_behavior");
    behavior_cfg_folder_ = fs::path(package_share_directory) / fs::path("config");
    behavior_cfg_ = YAML::LoadFile(behavior_cfg_folder_ / "behavior.yaml");

    global_blackboard_ = BT::Blackboard::create();
    initConfigurations();
    initHelperModules();
    initRosInterface();
    registerBtEnums(factory_);

    // Register the behavior tree
    using std::filesystem::directory_iterator;
    for (auto const& entry : directory_iterator(behavior_cfg_folder_)) {
      if (entry.path().extension() == ".xml") {
        factory_.registerBehaviorTreeFromFile(entry.path().string());
      }
    }
    motion_behavior_tree_ =
      factory_.createTree(behavior_cfg_["main_tree_name"].as<std::string>(), BT::Blackboard::create(global_blackboard_));
    head_motion_behavior_tree_ = factory_.createTree(behavior_cfg_["head_motion_tree_name"].as<std::string>(),
                                                     BT::Blackboard::create(global_blackboard_));
  }

  void Behavior::initConfigurations() {
    // Load configurationsnomadz_behavior_msgs
    game_settings_ = nomadz_configuration::getGameSettings();
    global_blackboard_->set<nomadz_configuration::GameSettings>("game_settings", game_settings_);
    global_blackboard_->set<nomadz_configuration::FieldLines>("field_lines", nomadz_configuration::FieldLines());
    global_blackboard_->set<YAML::Node>("behavior_cfg", behavior_cfg_);
  }

  void Behavior::initHelperModules() {
    target_pose_generator_ = std::make_shared<TargetPoseGenerator>(global_blackboard_);
    global_blackboard_->set<std::shared_ptr<TargetPoseGenerator>>("target_pose_generator", target_pose_generator_);
  }

  void Behavior::initRosInterface() {
    // Initialize ROS interface
    ros_interface_node_ = std::make_shared<nomadz_behavior::RosInterfaceNode>(
      "ros_interface_node", rclcpp::NodeOptions(), game_settings_, global_blackboard_);
    executor_.add_node(ros_interface_node_);
  }

  void Behavior::saveTreeModel() {
    std::string xml_models = BT::writeTreeNodesModelXML(factory_);
    fs::path tree_node_path = "src/nomadz_behavior/config/TreeNodesModel.xml";
    std::ofstream file(tree_node_path);
    if (file.is_open()) {
      file << xml_models;
      file.close();
      spdlog::info("Behavior tree model saved to TreeNodesModel.xml");
    } else {
      spdlog::error("Failed to save behavior tree model to TreeNodesModel.xml");
    }
  }

  void Behavior::run() {
#ifndef TARGET_ROBOT
    BT::Groot2Publisher publisher_motion_bt(motion_behavior_tree_);
    BT::Groot2Publisher publisher_head_bt(head_motion_behavior_tree_, 1677);
    saveTreeModel();
#endif
    rclcpp::Rate tick_rate(behavior_cfg_["bt_loop_frequency"].as<float>());
    while (rclcpp::ok()) {
      executor_.spin_some();
      motion_behavior_tree_.tickOnce();
      head_motion_behavior_tree_.tickOnce();
      ros_interface_node_->publish();
      tick_rate.sleep();
    }
  }
} // namespace nomadz_behavior
