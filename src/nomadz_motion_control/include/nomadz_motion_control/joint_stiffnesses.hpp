#pragma once

#include "nomadz_definitions/joint_indexes.hpp"

namespace nomadz_motion_control {
  constexpr int LEGACY_HARDNESS_DEFAULT = -1;
  constexpr float ACTIVE_HARDNESS_DEFAULT = 0.6F;

  struct JointStiffnesses {
    float mirror(nomadz_definitions::joint_indexes::JointIndexes joint_name) const {
      return values[nomadz_definitions::JOINT_NAME_MIRROR_MAP.at(joint_name)];
    }

    void mirror(const JointStiffnesses& other) {
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; i++) {
        values[i] = other.mirror(static_cast<nomadz_definitions::joint_indexes::JointIndexes>(i));
      }
    }

    JointStiffnesses mirror() const {
      JointStiffnesses mirrored_joint_stiffnesses_mirrored;
      mirrored_joint_stiffnesses_mirrored.mirror(*this);
      return mirrored_joint_stiffnesses_mirrored;
    }

    inline JointStiffnesses operator*(float scalar) const {
      JointStiffnesses joint_stiffnesses_scaled;
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; i++) {
        joint_stiffnesses_scaled.values[i] = values[i] * scalar;
      }
      return joint_stiffnesses_scaled;
    }

    inline JointStiffnesses operator+(const JointStiffnesses& other) const {
      JointStiffnesses joint_stiffnesses_sum;
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; i++) {
        joint_stiffnesses_sum.values[i] = values[i] + other.values[i];
      }
      return joint_stiffnesses_sum;
    }

    inline JointStiffnesses operator-(const JointStiffnesses& other) const {
      JointStiffnesses joint_stiffnesses_diff;
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; i++) {
        joint_stiffnesses_diff.values[i] = values[i] - other.values[i];
      }
      return joint_stiffnesses_diff;
    }

    inline void reset() {
      for (int i = 0; i < nomadz_definitions::joint_indexes::NUM_JOINTS; i++) {
        values[i] = LEGACY_HARDNESS_DEFAULT;
      }
    }

    std::array<float, nomadz_definitions::joint_indexes::NUM_JOINTS>
      values{}; // NOLINT(misc-non-private-member-variables-in-classes)
  };
} // namespace nomadz_motion_control
