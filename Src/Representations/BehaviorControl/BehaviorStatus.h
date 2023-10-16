/**
 * @file BehaviorStatus.h
 *
 * The file declares a class that containts data about the current behavior state.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Andreas Stolpmann
 */

#pragma once
#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"

/**
 * @class BehaviorStatus
 * A class that containts data about the current behavior state.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/behavior_status.hpp"
#endif
STREAMABLE_DECLARE(BehaviorStatus)

STREAMABLE_ROS(BehaviorStatus, {
public:
  ENUM(Role,

       striker,
       supporter,
       defender,
       keeper,

       visualRefereeChallenge,
       /* test roles */
       coach,
       dummy,
       testRole,
       dropIn);

  ENUM(TeamColor, red, blue);

  ENUM(Lost, no, yes);
  , FIELD_WRAPPER(TeamColor, red, nomadz_msgs::msg::BehaviorStatus::team_color, teamColor),
    FIELD_WRAPPER(Role, dummy, nomadz_msgs::msg::BehaviorStatus::role, role),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::BehaviorStatus::role_changed, roleChanged),
    FIELD_WRAPPER(Lost, no, nomadz_msgs::msg::BehaviorStatus::lost, lost),
    FIELD_WRAPPER(int, 2, nomadz_msgs::msg::BehaviorStatus::head_mode, headMode),
});
