/**
 * @file WalkRequest.h
 * This file declares a struct that represents a walk request.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>, Colin Graf and Oliver Urbann
 */

#pragma once

#include "Core/Math/Pose2D.h"
#include "Core/Enum.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @struct WalkRequest
 * A struct that represents a walk request.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/walk_request.hpp"
#endif
STREAMABLE_DECLARE(WalkRequest)

STREAMABLE_ROS(WalkRequest, {
public:
  // old engine{
  ENUM(Mode,
       speedMode,           /**< Interpret \c speed as absolute walking speed and ignore \c target. */
       percentageSpeedMode, /**< Interpret \c speed as percentage walking speed and ignore \c target. */
       targetMode, /**< Use \c target as walking target relative to the current position of the robot and interpret \c speed
                     as percentage walking speed. */
       patternMode);

  ENUM(Pattern, accurateKick, simpleKick);

  ENUM(KickType,
       nokick, /**< do not kick */
       left,   /**< kick using the left foot */
       right,  /**< kick using the right foot */
       sidewardsLeft,
       sidewardsRight);
  //}old engine

  ENUM(StandType, doubleSupport, leftSingleSupport, rightSingleSupport);

  ENUM(RequestType,
       speedAbs,    /**< Interpret \c speed as absolute walking speed and ignore \c target. */
       destination, /**< Interpret \c speed as percentage walking speed and ignore \c target. */
       ball, /**< Use \c target as walking target relative to the current position of the robot and interpret \c speed as
                percentage walking speed. */
       dribble);

  ENUM(StepRequest,
       none,
       previewKick,
       beginScript,    /**< Add custom step file after this marker */
       frontKickLeft,  /**< Kick forward with left foot */
       frontKickRight, /**< Kick forward with right foot */
       sideKickLeft,   /**< Kick with left side of left foot */
       sideKickRight   /**< Kick with right side of right foot */
  );

  bool isValid() const {
    return !std::isnan(static_cast<float>(request.rotation)) && !std::isnan(request.translation.x) &&
           !std::isnan(request.translation.y);
  }

  /**
   * \return \c true iff the current \c stepRequest is a kick.
   */
  bool isStepRequestKick() const {
    switch (stepRequest) {
    case frontKickLeft:
    case frontKickRight:
    case sideKickLeft:
    case sideKickRight:
      return true;
    case none:
    case previewKick:
    case beginScript:
    case numOfStepRequests:
      return false;
    }
    return false;
  }
  ,

    // Dortmund Engine

    FIELD_WRAPPER(RequestType, speedAbs, nomadz_msgs::msg::WalkRequest::request_type, requestType), /**< The walking mode. */
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::WalkRequest::request,
                          request), /**< Target relative to robot or speed in mm/s and radian/s. */
    FIELD_WRAPPER(StandType,
                  doubleSupport,
                  nomadz_msgs::msg::WalkRequest::stand_type,
                  standType), /**< How should the robot stand when speed is 0? */
    FIELD_WRAPPER(
      float, 0.f, nomadz_msgs::msg::WalkRequest::kick_strength, kickStrength), /**< Kick strength for Dortmund WE kick. */
    FIELD_WRAPPER(
      float, 0.f, nomadz_msgs::msg::WalkRequest::kick_direction, kickDirection), /**< Kick direction for Dortmund WE kick. */
    FIELD_WRAPPER(StepRequest,
                  none,
                  nomadz_msgs::msg::WalkRequest::step_request,
                  stepRequest), /**< The stepRequest (for in-walk predefined motions) */

    // Generic

    FIELD_WRAPPER(Mode, speedMode, nomadz_msgs::msg::WalkRequest::mode, mode), /**< The walking mode. */
    FIELD_WRAPPER_DEFAULT(
      Pose2D, nomadz_msgs::msg::WalkRequest::speed, speed), /**< Walking speeds, in percentage or mm/s and radian/s. */
    FIELD_WRAPPER_DEFAULT(
      Pose2D,
      nomadz_msgs::msg::WalkRequest::target,
      target), /**< Walking target, in mm and radians, relative to the robot. Use either a speed or a target. */
    FIELD_WRAPPER(KickType, right, nomadz_msgs::msg::WalkRequest::kick_type, kickType),
});
