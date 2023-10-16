/**
 * @file RefereePercept.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/referee_percept.hpp"
#endif

STREAMABLE_DECLARE(RefereePercept)

STREAMABLE_ROS(RefereePercept, {
  public : ENUM(RefereeSignal,
                noSignal,
                kickInBlueTeam,
                kickInRedTeam,
                goalKickBlueTeam,
                goalKickRedTeam,
                cornerKickBlueTeam,
                cornerKickRedTeam,
                goalBlueTeam,
                goalRedTeam,
                pushingFreeKickBlueTeam,
                pushingFreeKickRedTeam,
                fullTime,
                playerExchangeBlueTeam,
                playerExchangeRedTeam),
  FIELD_WRAPPER(int, -1, nomadz_msgs::msg::RefereePercept::signal, signal),
  FIELD_WRAPPER(int, -1, nomadz_msgs::msg::RefereePercept::gamecontroller_signal, gamecontrollerSignal),
  FIELD_WRAPPER(bool, false, nomadz_msgs::msg::RefereePercept::run_model, runModel),
  FIELD_WRAPPER(unsigned int,
                0,
                nomadz_msgs::msg::RefereePercept::last_time_game_controller_sent,
                lastTimeGameControllerSent), /**< Timestamp */
  FIELD_WRAPPER(float, 0.0, nomadz_msgs::msg::RefereePercept::pred_score, predScore),
  FIELD_WRAPPER_DEFAULT(std::vector<int>, nomadz_msgs::msg::RefereePercept::preds, preds),
});
