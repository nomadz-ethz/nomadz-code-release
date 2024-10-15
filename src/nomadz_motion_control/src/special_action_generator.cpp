#include "nomadz_motion_control/special_action_generator.hpp"

#include <stdexcept>
#include <filesystem>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "nomadz_core/math/constants.hpp"
#include "nomadz_motion_control/interpolation.hpp"
#include "nomadz_motion_control/mof_compiler.hpp"

namespace fs = std::filesystem;
namespace legacy_joint_indexes = nomadz_definitions::legacy_joint_indexes;

namespace nomadz_motion_control {
  SpecialActionGenerator::SpecialActionGenerator(const rclcpp::NodeOptions& options)
      : MotionBase("special_action_generator", options), param_listener_(get_node_parameters_interface()) {

    // NOTE(@naefjo): Execute the mof compiler and load the mof data.
    char buffer[1000];
    nomadz_motion_control::MofCompiler mof_compiler;
    if (mof_compiler.compileMotionFiles(buffer, sizeof(buffer))) {
      printf("Created 'config/specialActions.dat' successfully\n");
    } else {
      printf("%s", buffer);
      throw std::runtime_error("MofCompiler Failed.");
    }

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nomadz_motion_control");
    fs::path motion_net_data_path = fs::path(package_share_directory) / fs::path("config/special_actions.dat");
    motion_net_data_.loadFromFile(motion_net_data_path);

    special_action_mode_ = SpecialActionMode::ACTIVE;
    reset();
  }

  void SpecialActionGenerator::reset() {
    current_node_ = 0;
    last_joint_requests_.positions = current_joint_data_.positions;
    current_joint_requests_.positions = current_joint_data_.positions;
    special_action_finished_ = false;
    request_leave_ = false;
  }

  bool SpecialActionGenerator::getNextData(const short& special_action_type) {
    while (static_cast<MotionNetNode::NodeType>(motion_net_data_.node_vector[current_node_].data_row_[0]) !=
           MotionNetNode::DATA) {
      switch (static_cast<MotionNetNode::NodeType>(motion_net_data_.node_vector[current_node_].data_row_[0])) {
      case MotionNetNode::HARDNESS:
        last_joint_requests_.stiffnesses = current_joint_requests_.stiffnesses; // currentHardnessRequest;
        motion_net_data_.node_vector[current_node_].toJointStiffnesses(next_target_joint_requests_.stiffnesses,
                                                                       hardness_interpolation_length_);
        hardness_interpolation_counter_ = hardness_interpolation_length_;
        current_node_++;
        break;
      case MotionNetNode::CONDITIONAL_TRANSITION:
        if (static_cast<short>(motion_net_data_.node_vector[current_node_].data_row_[2]) != special_action_type) {
          current_node_++;
          special_action_finished_ = false;
          break;
        } else {
          special_action_finished_ = true;
        }
      // no break here: if condition is true, continue with transition!
      case MotionNetNode::TRANSITION:
        // follow transition
        if (current_node_ == 0) { // we come from extern
          current_node_ = motion_net_data_.label_extern_start[special_action_type];
        } else {
          current_node_ = static_cast<short>(motion_net_data_.node_vector[current_node_].data_row_[1]);
        }
        // leave if transition to external motion
        if (current_node_ == 0) {
          return false;
        }
        break;
      case MotionNetNode::DATA:
        break;
      }
    }
    motion_net_data_.node_vector[current_node_].toJointPositions(
      next_target_joint_requests_.positions, data_repetition_length_, interpolation_mode_, deshake_mode_);

    return true;
  }

  void SpecialActionGenerator::calculateJointRequest(const JointRequests& target_joint_requests,
                                                     JointRequests& joint_requests) {
    float ratio;
    // joint angles
    if (interpolation_mode_) {
      ratio = 1.F - static_cast<float>(data_repetition_counter_) / static_cast<float>(data_repetition_length_);
      joint_requests.positions =
        linearInterpolation<JointPositions>(last_joint_requests_.positions, target_joint_requests.positions, ratio);
    } else {
      joint_requests.positions = target_joint_requests.positions;
    }

    // hardness stuff
    if (hardness_interpolation_counter_ <= 0) {
      joint_requests.stiffnesses = target_joint_requests.stiffnesses;
    } else {
      ratio = 1.F - static_cast<float>(hardness_interpolation_counter_) / static_cast<float>(hardness_interpolation_length_);
      joint_requests.stiffnesses =
        linearInterpolation<JointStiffnesses>(last_joint_requests_.stiffnesses, target_joint_requests.stiffnesses, ratio);
    }
    joint_requests.joint_ignore = target_joint_requests.joint_ignore;
  }

  void SpecialActionGenerator::updateJointRequests() {
    auto params = param_listener_.get_params();
    auto speed_factor = static_cast<float>(params.speed_factor);

    if (special_action_mode_ != SpecialActionMode::DEACTIVE) {

      current_special_action_request_msg_ = current_motion_request_msg_.special_action_request;
      if (data_repetition_counter_ <= 0) {
        if (was_end_of_special_action_) {
          current_joint_requests_.stiffnesses.reset();
          next_target_joint_requests_.stiffnesses.reset();
        }
        was_end_of_special_action_ = !getNextData(current_special_action_request_msg_.special_action_type);
        // search next data, leave on transition to external motion
        if (was_end_of_special_action_) {
          return;
        }
        if (current_special_action_request_msg_.mirror) {
          next_target_joint_requests_ = next_target_joint_requests_.mirror();
        }
        data_repetition_counter_ = data_repetition_length_;

        // get currently executed special action from motion net traversal:
        executed_special_action_type_ =
          static_cast<short>(motion_net_data_.node_vector[current_node_].data_row_[legacy_joint_indexes::NUM_JOINTS + 3]);
        current_node_++;

      } else {
        data_repetition_counter_ -= static_cast<int>(nomadz_core::constants::MOTION_CYCLE_TIME * 1000 * speed_factor);
        hardness_interpolation_counter_ -= static_cast<int>(nomadz_core::constants::MOTION_CYCLE_TIME * 1000 * speed_factor);
      }

      // set current joint values
      JointRequests target_joint_requests = next_target_joint_requests_;
      processSpecialJointValues(target_joint_requests);

      calculateJointRequest(target_joint_requests, current_joint_requests_);

      // store value if current data line finished
      if (data_repetition_counter_ <= 0) {
        last_joint_requests_.positions = target_joint_requests.positions;
      }
    }
  }

  void SpecialActionGenerator::updateMotionInfoMsg() {
    motion_info_msg_.is_leaving_possible = was_end_of_special_action_;
    motion_info_msg_.is_motion_done = special_action_finished_;
    motion_info_msg_.odometry_offset = geometry_msgs::msg::Twist();
    if (was_end_of_special_action_) {
    } else {
      motion_info_msg_.executed_motion_request = current_motion_request_msg_;
      motion_info_msg_.executed_motion_request.special_action_request.special_action_type = executed_special_action_type_;
    }
  }

  bool SpecialActionGenerator::isLeavingPossible() {
    if (special_action_finished_) {
      request_leave_ = false;
    }
    return special_action_finished_;
  }

  void SpecialActionGenerator::processSpecialJointValues(JointRequests& joint_requests) {
    for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; ++i) {
      // This sensor feedback can drift since the stiffness has no effect on the simulation. Avoid legacy's "-" in general
      if (joint_requests.positions.values[i] == JOINT_OFF) {
        joint_requests.positions.values[i] = current_joint_data_.positions.values[i];
        joint_requests.stiffnesses.values[i] = 0.F;
        joint_requests.joint_ignore[i] = false;
      } else if (joint_requests.positions.values[i] == JOINT_IGNORE) {
        joint_requests.positions.values[i] = current_joint_data_.positions.values[i];
        joint_requests.joint_ignore[i] = true;
      } else {
        joint_requests.joint_ignore[i] = false;
      }
    }
  }

} // namespace nomadz_motion_control
