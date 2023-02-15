/**
 * @file Representations/MotionControl/RobotBalance.h
 *
 * This file declares a class that represents information about the robot balance.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Felix Wenk
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @class RobotBalance
 * A class that represents information about the robot balance.
 */
#include "Core/Streams/FieldWrapper.h"

STREAMABLE_DECLARE_LEGACY(RobotBalance)

STREAMABLE_LEGACY(RobotBalance, {
  public : enum {maxZmpPreview = 100}, /**< Maximum number of previewed ZMPs sent in this representation. */
  FIELD_WRAPPER_LEGACY(
    int, 1, nomadz_msgs::msg::RobotBalance::num_zmp_preview, numZmpPreview), /**< Number of actually previewed ZMPs. */
  (Vector2<>[maxZmpPreview])zmpPreview, /**< Previewed ZMPs. zmpPreview[0] is the currently demanded ZMP. */
  FIELD_WRAPPER_DEFAULT_LEGACY(Vector2<>,
                               nomadz_msgs::msg::RobotBalance::zmp,
                               zmp), /**< The currently estimated zero-moment point. */
  FIELD_WRAPPER_LEGACY(bool,
                       false,
                       nomadz_msgs::msg::RobotBalance::left_support,
                       leftSupport), /**< True of the robot stands on its left foot. */
});
