#include "nomadz_motion_control/motion_net_data.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include "nomadz_motion_control/joint_stiffnesses.hpp"

namespace joint_indexes = nomadz_definitions::joint_indexes;
namespace legacy_joint_indexes = nomadz_definitions::legacy_joint_indexes;
using nomadz_definitions::JOINT_NAMES_TO_LEGACY_MAP;

namespace nomadz_motion_control {
  void MotionNetNode::toJointPositions(JointPositions& joint_positions,
                                       int& data_repetition_counter,
                                       bool& interpolation_mode,
                                       bool& deshake_mode) {
    for (int i = 0; i < joint_indexes::NUM_JOINTS; ++i) {
      joint_positions.values[i] = data_row_[JOINT_NAMES_TO_LEGACY_MAP.at(static_cast<joint_indexes::JointIndexes>(i)) + 1];
      if (joint_positions.values[i] != JOINT_OFF && joint_positions.values[i] != JOINT_IGNORE) {
        joint_positions.values[i] *= M_PI / 180.F;
      }
    }
    data_repetition_counter = static_cast<int>(data_row_[legacy_joint_indexes::NUM_JOINTS + 2]);
    interpolation_mode = (static_cast<int>(data_row_[legacy_joint_indexes::NUM_JOINTS + 1]) & 1) != 0;
    deshake_mode = (static_cast<int>(data_row_[legacy_joint_indexes::NUM_JOINTS + 1]) & 2) != 0;
  }

  void MotionNetNode::toJointStiffnesses(JointStiffnesses& joint_stiffnesses, int& hardness_interpolation_length) {
    for (int i = 0; i < joint_indexes::NUM_JOINTS; i++) {
      joint_stiffnesses.values[i] =
        static_cast<float>(data_row_[JOINT_NAMES_TO_LEGACY_MAP.at(static_cast<joint_indexes::JointIndexes>(i)) + 1]);

      if (joint_stiffnesses.values[i] == LEGACY_HARDNESS_DEFAULT) {
        joint_stiffnesses.values[i] = ACTIVE_HARDNESS_DEFAULT;
      } else {
        joint_stiffnesses.values[i] /= 100.F;
      }
    }
    hardness_interpolation_length = static_cast<int>(data_row_[legacy_joint_indexes::NUM_JOINTS + 1]);
  }

  void MotionNetData::loadFromFile(const std::string filename) {
    std::ifstream file(filename);
    std::string line;
    std::string word;
    int num_nodes;
    int node_index = -2;

    if (!file.is_open()) {
      throw std::runtime_error("Could not open file " + filename);
    }
    while (std::getline(file, line)) {
      if (line[0] == '/' || line[0] == '\0') {
        continue;
      }
      std::istringstream iss(line);
      std::string word;
      if (node_index == -2) {
        for (int i = 0; i < NUM_SPECIAL_ACTIONS; ++i) {
          iss >> label_extern_start[i];
        }
      } else if (node_index == -1) {
        iss >> num_nodes;
        node_vector.resize(num_nodes);
      } else {
        short s;
        iss >> s;

        switch (s) {
        case 1:
          node_vector[node_index].data_row_[0] = MotionNetNode::CONDITIONAL_TRANSITION;
          iss >> node_vector[node_index].data_row_[1] >> node_vector[node_index].data_row_[2] >>
            node_vector[node_index].data_row_[legacy_joint_indexes::NUM_JOINTS + 3];
          break;
        case 2:
          node_vector[node_index].data_row_[0] = MotionNetNode::TRANSITION;
          iss >> node_vector[node_index].data_row_[1] >>
            node_vector[node_index].data_row_[legacy_joint_indexes::NUM_JOINTS + 3];
          break;
        case 3:
          node_vector[node_index].data_row_[0] = MotionNetNode::DATA;
          for (int j = 1; j < legacy_joint_indexes::NUM_JOINTS + 4; ++j) {
            iss >> node_vector[node_index].data_row_[j];
          }
          break;
        case 4:
          node_vector[node_index].data_row_[0] = MotionNetNode::HARDNESS;
          for (int j = 1; j < legacy_joint_indexes::NUM_JOINTS + 3; j++) {
            iss >> node_vector[node_index].data_row_[j];
          }
          break;
        }
      }
      node_index++;
    }
    file.close();
  }
} // namespace nomadz_motion_control
