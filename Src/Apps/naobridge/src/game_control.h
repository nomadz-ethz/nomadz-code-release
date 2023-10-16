/**
 * @file game_ctrl.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#ifndef GAME_CTRL_H
#define GAME_CTRL_H

#include <arpa/inet.h>

#include "Core/Communication/RoboCupControlData.h"
#include "Core/System/UdpComm.h"

#include "naobridge/nomadz.h"

enum LED {
  chestRed,
  chestGreen,
  chestBlue,
  leftFootRed,
  leftFootGreen,
  leftFootBlue,
  rightFootRed,
  rightFootGreen,
  rightFootBlue,
  numOfLEDs
};

// game control functions
void ctrl_init();
void ctrl_close();

// game control output
void ctrl_handle_output(float* actuators, const RoboCup::RoboCupGameControlData& gameCtrlData);
void ctrl_set_led(float* actuators, LBHActuatorIds led, float red, float green, float blue);
bool ctrl_send();

// game control handle input
bool ctrl_receive(RoboCup::RoboCupGameControlData& gameCtrlData);
void ctrl_handle_input(const float* sensors,
                       RoboCup::RoboCupGameControlData& gameCtrlData,
                       bool active,
                       bool resetToInitial);

#endif
