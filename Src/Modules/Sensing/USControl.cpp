/**
 * @file USControl.cpp
 *
 * Implementation of a module that controls the firing strategy
 * of the ultrasound sensors.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "USControl.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"
#include "Core/Streams/InStreams.h"

USControl::USControl() : lastSendTime(0), lastSwitchTime(0), currentMode(0), commandSent(false) {}

void USControl::update(USRequest& usRequest) {
  // FIXME: do not update us request as it does not work
  usRequest.sendMode = -1;
  usRequest.receiveMode = 1;
  commandSent = false;
  return;

  if (stopOnPlayDead && theMotionRequest.motion == MotionRequest::specialAction &&
      theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead) {
    usRequest.sendMode = -1;
    usRequest.receiveMode = -1;
    commandSent = false;
  } else if (commandSent) {
    usRequest.sendMode = -1;
    if (Time::getTimeSince(lastSendTime) >= timeBetweenSendAndReceive) {
      usRequest.receiveMode = modes[currentMode];
      commandSent = false;
    }
    // wait a while longer
  } else {
    usRequest.receiveMode = -1; // do not read anything

    // if a command has been sent last frame: check if we should send one this frame
    if (Time::getTimeSince(lastSendTime) >= sendInterval) {
      if (Time::getTimeSince(lastSwitchTime) >= switchInterval) {
        currentMode = (currentMode + 1) % modes.size();
        lastSwitchTime = Time::getCurrentSystemTime();
      }
      usRequest.sendMode = modes[currentMode];
      lastSendTime = Time::getCurrentSystemTime();
      commandSent = true;
    } else {
      usRequest.sendMode = -1; // do not send anything
    }
  }
}

MAKE_MODULE(USControl, Sensing)
