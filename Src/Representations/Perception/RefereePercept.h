/**
 * @file RefereePercept.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz/msg/referee_percept.hpp"
#endif

STREAMABLE_DECLARE(RefereePercept)

STREAMABLE_ROS(RefereePercept, {
  public : ENUM(RefereeSignal,
                noSignal,
                kickInRedTeam,
                kickInBlueTeam,
                goalKickRedTeam,
                goalKickBlueTeam,
                cornerKickRedTeam,
                cornerKickBlueTeam,
                goalRedTeam,
                goalBlueTeam,
                pushingFreeKickRedTeam,
                pushingFreeKickBlueTeam,
                fullTime),
  FIELD_WRAPPER(int, -1, nomadz_msgs::msg::RefereePercept::referee_signal, signal),
  FIELD_WRAPPER(int, -1, nomadz_msgs::msg::RefereePercept::confidence, confidence),
});
