/**
 * @file OwnSideModel.h
 *
 * The file implements a model that states that the robot cannot have left its own
 * side since the last kick-off and how far it can have gotten along the field.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

/**
 * @class OwnSideModel
 * A model that states that the robot cannot have left its own
 * side since the last kick-off and how far it can have gotten along
 * the field.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/own_side_model.hpp"
#endif
STREAMABLE_DECLARE(OwnSideModel)

STREAMABLE_ROS(
  OwnSideModel,
  {
    ,
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::OwnSideModel::still_in_own_side,
                  stillInOwnSide), /**< The robot must still be in its own side. */
    FIELD_WRAPPER(float,
                  100000.f,
                  nomadz_msgs::msg::OwnSideModel::largest_x_possible,
                  largestXPossible), /**< The largest x-coordinate that is currently possible. */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::OwnSideModel::return_from_game_controller_penalty,
                  returnFromGameControllerPenalty), /**< The robot was unpenalized by the GameController and believes it. */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::OwnSideModel::return_from_manual_penalty,
                  returnFromManualPenalty), /**< The robot was unpenalized by the GameController and believes it. */
  });
