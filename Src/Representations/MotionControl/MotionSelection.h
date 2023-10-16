/**
 * @file MotionSelection.h
 *
 * This file declares a class that represents the motions actually selected based on the constraints given.
 *
 *  This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "MotionRequest.h"
#include <cstring>

/**
 * @class MotionSelection
 * A class that represents the motions actually selected based on the constraints given.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/motion_selection.hpp"
#endif
STREAMABLE_DECLARE(MotionSelection)

STREAMABLE_ROS(MotionSelection, {
public:
  ENUM(ActivationMode, deactive, active, first)

  // FIXME: copy from MotionRequest
  ENUM_ALIAS(MotionRequest, Motion, specialAction, walk, kick);
  ,
    FIELD_WRAPPER(MotionRequest::Motion,
                  MotionRequest::specialAction,
                  nomadz_msgs::msg::MotionSelection::target_motion,
                  targetMotion), /**< The motion that is the destination of the current interpolation. */
    FIELD_WRAPPER(ActivationMode,
                  active,
                  nomadz_msgs::msg::MotionSelection::special_action_mode,
                  specialActionMode), /**< Whether and how the special action module is active. */
    FIELD_WRAPPER_DEFAULT(float[MotionRequest::numOfMotions],
                          nomadz_msgs::msg::MotionSelection::ratios,
                          ratios), /**< The current ratio of each motion in the final joint request. */
    FIELD_WRAPPER_DEFAULT(SpecialActionRequest,
                          nomadz_msgs::msg::MotionSelection::special_action_request,
                          specialActionRequest), /**< The special action request, if it is an active motion. */
    FIELD_WRAPPER_DEFAULT(WalkRequest,
                          nomadz_msgs::msg::MotionSelection::walk_request,
                          walkRequest), /**< The walk request, if it is an active motion. */

    // Initialization
    memset(ratios, 0, sizeof(ratios));
  ratios[MotionRequest::specialAction] = 1;
});
