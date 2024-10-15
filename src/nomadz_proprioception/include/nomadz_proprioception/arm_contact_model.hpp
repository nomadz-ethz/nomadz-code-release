#pragma once

#include <rclcpp/rclcpp.hpp>

#include "nomadz_proprioception/proprioception_enums.hpp"
#include "nomadz_proprioception_msgs/msg/arm_contact_model.hpp"

namespace nomadz_proprioception {

  struct ArmContactModel {
    PushDirection push_direction_left;  // NOLINT(misc-non-private-member-variables-in-classes)
    PushDirection push_direction_right; // NOLINT(misc-non-private-member-variables-in-classes)
    bool contact_left;                  // NOLINT(misc-non-private-member-variables-in-classes)
    bool contact_right;                 // NOLINT(misc-non-private-member-variables-in-classes)
    rclcpp::Duration duration_left =
      rclcpp::Duration::from_seconds(0); // NOLINT(misc-non-private-member-variables-in-classes)
    rclcpp::Duration duration_right =
      rclcpp::Duration::from_seconds(0); // NOLINT(misc-non-private-member-variables-in-classes)
  };

} // namespace nomadz_proprioception
