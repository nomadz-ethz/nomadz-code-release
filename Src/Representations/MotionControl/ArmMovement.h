/**
 * @file ArmMovement.h
 *
 * This file declares a class that represents the output of ArmAnimator.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Core/Math/Pose2D.h"

/**
 * @class ArmMovement
 * A class that represents the output of the walking engine.
 */

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/arm_movement.hpp"
#endif
STREAMABLE_DECLARE(ArmMovement)

STREAMABLE_ROS(
  ArmMovement,
  {
    ,
    FIELD_WRAPPER_DEFAULT(JointData, nomadz_msgs::msg::ArmMovement::joint_angles, jointAngles),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::ArmMovement::usearms, usearms),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::ArmMovement::arms_in_contact_avoidance, armsInContactAvoidance),
  });
