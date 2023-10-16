/**
 * @file ArmContactModel.h
 *
 * Declaration of class ArmContactModel.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>,
 * <a href="mailto:simont@informatik.uni-bremen.de">Simon Taddiken</a> and
 * <a href="mailto:arneboe@informatik.uni-bremen.de">Arne Böckmann</a>
 */

#pragma once

#include "Core/Enum.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @class ArmContactModel
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/arm_contact_model.hpp"
#endif
STREAMABLE_DECLARE(ArmContactModel)

STREAMABLE_ROS(ArmContactModel, {
  public : ENUM(PushDirection,
                N,   /**< Arm is being pushed to the front */
                S,   /**< Arm is being pushed to the back */
                W,   /**< Arm is being pushed to the left */
                E,   /**< Arm is being pushed to the right */
                NW,  /**< Arm is being pushed to the front and the left */
                NE,  /**< Arm is being pushed to the front and the right */
                SW,  /**< Arm is being pushed to the back and the left */
                SE,  /**< Arm is being pushed to the back and the right */
                NONE /**< If no contact is detected */
                ),
  FIELD_WRAPPER(bool,
                false,
                nomadz_msgs::msg::ArmContactModel::contact_left,
                contactLeft), /**< The contact state of the robot's left arm. */
  FIELD_WRAPPER(bool,
                false,
                nomadz_msgs::msg::ArmContactModel::contact_right,
                contactRight), /**< The contact state of the robot's right arm. */

  // only evaluate these values if contactLeft or contactRight is true */

  FIELD_WRAPPER(PushDirection,
                NONE,
                nomadz_msgs::msg::ArmContactModel::push_direction_left,
                pushDirectionLeft), /**< direction in which the left arm is being pushed */
  FIELD_WRAPPER(PushDirection,
                NONE,
                nomadz_msgs::msg::ArmContactModel::push_direction_right,
                pushDirectionRight), /**< direction in which the right arm is being pushed */

  // only evaluate these values if contactLeft or contactRight is true

  FIELD_WRAPPER(PushDirection,
                NONE,
                nomadz_msgs::msg::ArmContactModel::last_push_direction_left,
                lastPushDirectionLeft), /**< direction in which the left arm was last pushed */
  FIELD_WRAPPER(PushDirection,
                NONE,
                nomadz_msgs::msg::ArmContactModel::last_push_direction_right,
                lastPushDirectionRight), /**< direction in which the right arm was last pushed */

  // The duration of the push in motion frames (100fps = 1s).

  FIELD_WRAPPER(unsigned, 0, nomadz_msgs::msg::ArmContactModel::duration_left, durationLeft),
  FIELD_WRAPPER(unsigned, 0, nomadz_msgs::msg::ArmContactModel::duration_right, durationRight),
  FIELD_WRAPPER(unsigned, 0, nomadz_msgs::msg::ArmContactModel::time_of_last_contact_left, timeOfLastContactLeft),
  FIELD_WRAPPER(unsigned, 0, nomadz_msgs::msg::ArmContactModel::time_of_last_contact_right, timeOfLastContactRight),
});
