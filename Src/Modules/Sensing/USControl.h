/**
 * @file USControl.h
 *
 * Declaration of a module that controls the firing strategy
 * of the ultrasound sensors.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/USRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

MODULE(USControl)
REQUIRES(MotionRequest);
PROVIDES_WITH_MODIFY_AND_OUTPUT(USRequest)
LOADS_PARAMETER(int, sendInterval)              /**< Time to wait between sending two requests (in ms). */
LOADS_PARAMETER(int, switchInterval)            /**< Time to wait until switching to the next firing mode (in ms). */
LOADS_PARAMETER(int, timeBetweenSendAndReceive) /**< time to wait between send an receive command (in ms). */
LOADS_PARAMETER(std::vector<int>,
                modes)                /**< The available firing modes (according to NAOqi DCM ultrasound documentation). */
LOADS_PARAMETER(bool, stopOnPlayDead) /**< Stop firing when the playDead special action is active? */
END_MODULE

/**
 * @class USControl
 * A module that controls the firing strategy of the ultrasound sensors.
 */
class USControl : public USControlBase {
private:
  unsigned lastSendTime;   /**< The time when the last ultrasonic wave was send. */
  unsigned lastSwitchTime; /**< The time when the used transmitter was changed. */
  unsigned currentMode;    /**< The index of the transmitter mode that is currently active. */
  bool commandSent;        /**< True if a command was sent to naobridge one frame ago */

  void update(USRequest& usRequest);

public:
  USControl();
};
