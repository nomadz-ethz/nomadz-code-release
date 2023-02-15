/**
 * @file SpecialActionRequest.h
 *
 * This file declares a class to represent special action requests.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"

/**
 * @class SpecialActionRequest
 * The class represents special action requests.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/special_action_request.hpp"
#endif
STREAMABLE_DECLARE(SpecialActionRequest)

STREAMABLE_ROS(SpecialActionRequest, {
public:
  /** ids for all special actions */
  ENUM(SpecialActionID,
       playDead,
       stand,
       standWide,
       standHigh,
       getUpBackNao,
       getUpBackNao22,
       getUpFrontNao,
       getUpFrontNao22,
       sitDown,
       sitDownKeeper,
       sumoPosition,
       goUp,
       kickSidewardsNao,
       keeperJumpLeft,
       keeperStandJumpLeft,
       keeperJumpLeftPenalty,
       keeperJumpLeftSign,
       afterGenuflect,
       kickDiagonalNao,
       genuflect,
       keeperJumpLeftSim,
       stopBall,
       fallprotectionback,
       fallprotectionfront,
       fallprotectionside,
       jointCalibrationSit,
       jointCalibrationStand,
       jointCalibrationStandStiff,
       Ref_StandInit,
       Ref_KickInRight,
       Ref_KickInLeft,
       Ref_GoalKickRight,
       Ref_GoalKickLeft,
       Ref_CornerKickRight,
       Ref_CornerKickLeft,
       Ref_GoalRight,
       Ref_GoalLeft,
       Ref_PushingFreeKickRight,
       Ref_PushingFreeKickLeft,
       Ref_FullTime);

  /**
   * The function searches the id for a special action name.
   * @param name The name of the special action.
   * @return The corresponding id if found, or numOfSpecialActions if not found.
   */
  static SpecialActionID getSpecialActionFromName(const char*name),
    FIELD_WRAPPER(SpecialActionID,
                  playDead,
                  nomadz_msgs::msg::SpecialActionRequest::special_action,
                  specialAction),                                                       /**< The special action selected. */
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::SpecialActionRequest::mirror, mirror), /**< Mirror left and right. */
});
