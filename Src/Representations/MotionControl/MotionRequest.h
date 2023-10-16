/**
 * @file MotionRequest.h
 *
 * This file declares a class that represents the motions that can be requested from the robot.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "SpecialActionRequest.h"
#include "WalkRequest.h"
#include "KickRequest.h"

class CameraInfo;
class CameraMatrix;

/**
 * @class MotionRequest
 * A class that represents the motions that can be requested from the robot.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/motion_request.hpp"
#endif
STREAMABLE_DECLARE(MotionRequest)

STREAMABLE_ROS(MotionRequest, {
public:
  ENUM(Motion, specialAction, walk, kick);

  /**
   * Prints the motion request to a readable string. (E.g. "walk: 100mm/s 0mm/s 0°/s")
   * @param destination The string to fill
   */
  void printOut(char* destination) const;

  /** Draws something*/
  void draw() const;

  void drawOnImage(const CameraMatrix&cameraMatrix, const CameraInfo&cameraInfo) const,
    FIELD_WRAPPER(Motion, specialAction, nomadz_msgs::msg::MotionRequest::motion, motion), /**< The selected motion. */
    FIELD_WRAPPER_DEFAULT(SpecialActionRequest,
                          nomadz_msgs::msg::MotionRequest::special_action_request,
                          specialActionRequest), /**< The special action request, if it is the selected motion. */
    FIELD_WRAPPER_DEFAULT(WalkRequest,
                          nomadz_msgs::msg::MotionRequest::walk_request,
                          walkRequest), /**< The walk request, if it is the selected motion. */
    FIELD_WRAPPER_DEFAULT(KickRequest,
                          nomadz_msgs::msg::MotionRequest::kick_request,
                          kickRequest), /**< The kick request, if it is the selected motion. */
});

/**
 * Input for Request Translator, either from path or directly from motion request
 */
STREAMABLE_ALIAS(SpeedRequest, Pose2D, {
public:
  ALIAS_ROS_HELPERS(SpeedRequest, Pose2D)
  SpeedRequest() = default;
  /** Draws something (relative coordinates) */
  void draw() const;
});
