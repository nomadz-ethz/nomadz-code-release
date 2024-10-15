#pragma once

namespace nomadz_behavior {
  struct RobotPosture {
    bool is_standing;
    bool is_on_ground;
    bool has_ground_contact;
    enum class FallDirection { NONE, FRONT, BACK, LEFT, RIGHT };
    FallDirection fall_direction;
  };

  struct ArmContact {
    bool left_arm_contact;
    bool right_arm_contact;
  };

} // namespace nomadz_behavior
