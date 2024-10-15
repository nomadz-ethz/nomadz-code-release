#pragma once

#include <array>
#include <string_view>

namespace nomadz_definitions {
  namespace side {
    enum Side { LEFT, RIGHT, NUM_SIDES };

    inline Side mirror(const Side side) {
      return side == LEFT ? RIGHT : LEFT;
    }
  } // namespace side

  namespace limbs {
    enum Limbs {
      NECK = 0,
      HEAD,
      SHOULDER_LEFT,
      BICEPS_LEFT,
      ELBOW_LEFT,
      FOREARM_LEFT,
      WRIST_LEFT,
      SHOULDER_RIGHT,
      BICEPS_RIGHT,
      ELBOW_RIGHT,
      FOREARM_RIGHT,
      WRIST_RIGHT,
      PELVIS_LEFT,
      HIP_LEFT,
      THIGH_LEFT,
      TIBIA_LEFT,
      ANKLE_LEFT,
      FOOT_LEFT,
      PELVIS_RIGHT,
      HIP_RIGHT,
      THIGH_RIGHT,
      TIBIA_RIGHT,
      ANKLE_RIGHT,
      FOOT_RIGHT,
      TORSO,
      NUM_LIMBS
    };

    /** @brief Get the parent limb of a given limb
     *
     * The parent limb is the previous limb in the kinematic chain
     * e.g. the parent of the left bicep is the left shoulder,
     * the parent of the left pelvis is the tors, etc.
     *
     * @param limb The limb to get the parent of
     * @return The parent limb
     */
    constexpr Limbs getLimbParent(Limbs limb) {
      // clang-format off
      constexpr std::array limb_to_parent = {
        TORSO,
        NECK,
        TORSO,
        SHOULDER_LEFT,
        BICEPS_LEFT,
        ELBOW_LEFT,
        FOREARM_LEFT,
        TORSO,
        SHOULDER_RIGHT,
        BICEPS_RIGHT,
        ELBOW_RIGHT,
        FOREARM_RIGHT,
        TORSO,
        PELVIS_LEFT,
        HIP_LEFT,
        THIGH_LEFT,
        TIBIA_LEFT,
        ANKLE_LEFT,
        TORSO,
        PELVIS_RIGHT,
        HIP_RIGHT,
        THIGH_RIGHT,
        TIBIA_RIGHT,
        ANKLE_RIGHT
      };
      // clang-format on

      return static_cast<Limbs>(limb_to_parent[limb]);
    }

    namespace detail {
      // clang-format off
      constexpr std::array LIMB_TO_NAME = {
        "neck",
        "head",
        "l_shoulder",
        "l_bicep",
        "l_elbow",
        "l_fore_arm",
        "l_wrist",
        "r_shoulder",
        "r_bicep",
        "r_elbow",
        "r_fore_arm",
        "r_wrist",
        "l_pelvis",
        "l_hip",
        "l_thigh",
        "l_tibia",
        "l_ankle_pitch",
        "l_ankle",
        "r_pelvis",
        "r_hip",
        "r_thigh",
        "r_tibia",
        "r_ankle_pitch",
        "r_ankle",
        "torso",
      };
      // clang-format on
    } // namespace detail

    /** @brief Get the name of a limb
     *
     * NOTE: the names match the names used in the nao_simple
     * URDF in our fork of the nao_description repository.
     *
     * @param limb The limb to get the name of
     * @return The name of the limb
     */
    constexpr const char* getLimbName(Limbs limb) {
      return detail::LIMB_TO_NAME[limb];
    }

    inline Limbs getLimbByName(std::string_view name) {
      for (size_t i = 0; i < NUM_LIMBS; ++i) {
        if (name == detail::LIMB_TO_NAME[i]) {
          return static_cast<Limbs>(i);
        }
      }
      return NUM_LIMBS;
    }
  } // namespace limbs
} // namespace nomadz_definitions
