/**
 * @file HeadMotionRequest.h
 *
 * This file declares a class that represents the requested head motion.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Core/Math/Vector3.h"
#include "Core/Enum.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @class HeadMotionRequest
 * A class that represents the requested head motion.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/head_motion_request.hpp"
#endif
STREAMABLE_DECLARE(HeadMotionRequest)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/team_head_control_state.hpp"
#endif
STREAMABLE_DECLARE(TeamHeadControlState)

STREAMABLE_ROS(HeadMotionRequest, {
public:
  ENUM(Mode,
       panTiltMode,       /**< Use \c pan, \c tilt and \c speed. */
       targetMode,        /**< (A target relative to the center of hip.) Use \c target and \c speed. */
       targetOnGroundMode /**< Use \c target and \c speed. */
  );

  ENUM(CameraControlMode, autoCamera, lowerCamera, upperCamera)
  , FIELD_WRAPPER(Mode, panTiltMode, nomadz_msgs::msg::HeadMotionRequest::mode, mode), /**< The active head motion mode. */
    FIELD_WRAPPER(CameraControlMode,
                  lowerCamera,
                  nomadz_msgs::msg::HeadMotionRequest::camera_control_mode,
                  cameraControlMode), /**< The active camera control mode. */
    FIELD_WRAPPER(
      bool, false, nomadz_msgs::msg::HeadMotionRequest::watch_field, watchField), /**< True, if as much as possible of
                                        the field should be watched instead of centering the target in the image. */
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::HeadMotionRequest::pan, pan),       /**< Head pan target angle in radians. */
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::HeadMotionRequest::tilt, tilt),     /**< Head tilt target angle in radians. */
    FIELD_WRAPPER(float,
                  1,
                  nomadz_msgs::msg::HeadMotionRequest::speed,
                  speed), /**< Maximum joint speed to reach target angles in radians/s. */
    FIELD_WRAPPER_DEFAULT(
      Vector3<>, nomadz_msgs::msg::HeadMotionRequest::target, target), /**< Look at target relative to the robot. */
});

STREAMABLE_ROS(TeamHeadControlState,
               {
                 ,
                 FIELD_WRAPPER(bool, false, nomadz_msgs::msg::TeamHeadControlState::checks_ball, checksBall),
                 FIELD_WRAPPER(bool, false, nomadz_msgs::msg::TeamHeadControlState::uses_active_vision, usesActiveVision),
               });
