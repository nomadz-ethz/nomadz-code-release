#pragma once

#include <array>
#include <vector>
#include <string>

#include "nomadz_definitions/joint_indexes.hpp"
#include "nomadz_motion_control/joint_requests.hpp"
#include "nomadz_motion_control_msgs/special_action_request_enums.hpp"

namespace nomadz_motion_control {
  struct MotionNetNode {
    enum NodeType { CONDITIONAL_TRANSITION, TRANSITION, DATA, HARDNESS };

    void toJointPositions(JointPositions& joint_positions,
                          int& data_repetition_counter,
                          bool& interpolation_mode,
                          bool& deshake_mode);

    void toJointStiffnesses(JointStiffnesses& joint_stiffnesses, int& hardness_interpolation_length);

    std::array<float, nomadz_definitions::legacy_joint_indexes::NUM_JOINTS + 4>
      data_row_; // NOLINT(misc-non-private-member-variables-in-classes)
  };

  struct MotionNetData {
    static const int NUM_SPECIAL_ACTIONS =
      static_cast<int>(nomadz_motion_control_msgs::SpecialActionType::NUM_SPECIAL_ACTIONS);

    MotionNetData() = default;

    void loadFromFile(std::string filename);

    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    short label_extern_start[NUM_SPECIAL_ACTIONS + 1];
    std::vector<MotionNetNode> node_vector;
    // NOLINTEND(misc-non-private-member-variables-in-classes)
  };
} // namespace nomadz_motion_control
