#pragma once

#include "nomadz_definitions/joint_indexes.hpp"

namespace nomadz_motion_control {
  constexpr float JOINT_OFF = 1000.F;
  constexpr float JOINT_IGNORE = 2000.F;

  struct JointPositions {
    float mirror(nomadz_definitions::joint_indexes::JointIndexes joint_name) const {
      nomadz_definitions::joint_indexes::JointIndexes flipped_joint_name =
        nomadz_definitions::JOINT_NAME_MIRROR_MAP.at(joint_name);

      const bool is_pitch_joint =
        std::find(nomadz_definitions::PITCH_JOINT_NAMES.begin(), nomadz_definitions::PITCH_JOINT_NAMES.end(), joint_name) !=
        nomadz_definitions::PITCH_JOINT_NAMES.end();
      float mirroring_sign = is_pitch_joint ? 1.F : -1.F;
      mirroring_sign =
        (values[flipped_joint_name] == JOINT_OFF || values[flipped_joint_name] == JOINT_IGNORE) ? 1.F : mirroring_sign;
      return mirroring_sign * values[flipped_joint_name];
    }

    void mirror(const JointPositions& other) {
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; i++) {
        values[i] = other.mirror(static_cast<nomadz_definitions::joint_indexes::JointIndexes>(i));
      }
    }

    JointPositions mirror() const {
      JointPositions joint_positions_mirrored;
      joint_positions_mirrored.mirror(*this);
      return joint_positions_mirrored;
    }

    inline JointPositions operator*(float scalar) const {
      JointPositions joint_positions_scaled;
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; i++) {
        joint_positions_scaled.values[i] = values[i] * scalar;
      }
      return joint_positions_scaled;
    }

    inline JointPositions operator+(const JointPositions& other) const {
      JointPositions joint_positions_sum;
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; i++) {
        joint_positions_sum.values[i] = values[i] + other.values[i];
      }
      return joint_positions_sum;
    }

    inline JointPositions operator-(const JointPositions& other) const {
      JointPositions joint_positions_diff;
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; i++) {
        joint_positions_diff.values[i] = values[i] - other.values[i];
      }
      return joint_positions_diff;
    }

    std::array<float, nomadz_definitions::joint_indexes::NUM_JOINTS>
      values{}; // NOLINT(misc-non-private-member-variables-in-classes)
  };
} // namespace nomadz_motion_control
