/**
 * @file LEDHandler.cpp
 *
 * This file implements a module that generates the LEDRequest from certain representations.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include "LEDHandler.h"
#include <algorithm>

void LEDHandler::update(LEDRequest& ledRequest) {
  // reset
  for (int i = 0; i < ledRequest.numOfLEDs; ++i) {
    ledRequest.ledStates[i] = LEDRequest::off;
  }

  // update
  if (eyeSwirlIterationCounter >= 3) {
    eyeSwirlIterationCounter = 0;
    turningPositionCounter++;
  } else {
    eyeSwirlIterationCounter++;
  }
  setRightEar(ledRequest);
  setLeftEar(ledRequest);
  setRightEye(ledRequest);
  setLeftEye(ledRequest);
  setChestButton(ledRequest);
}

void LEDHandler::setRightEar(LEDRequest& ledRequest) {
  // right ear -> battery

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEar];

  int onLEDs = std::min((int)(theFilteredSensorData.data[FilteredSensorData::batteryLevel] / 0.1f), 9);

  for (int i = 0; i <= onLEDs; i++) {
    ledRequest.ledStates[LEDRequest::earsRight0Deg + i] = state;
  }
}

void LEDHandler::setLeftEar(LEDRequest& ledRequest) {
  if (theTeamMateData.numOfConnectedTeamMates > 0) {
    ledRequest.ledStates[LEDRequest::earsLeft0Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft36Deg] = LEDRequest::on;
  }
  if (theTeamMateData.numOfConnectedTeamMates > 1) {
    ledRequest.ledStates[LEDRequest::earsLeft72Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft108Deg] = LEDRequest::on;
  }
  if (theTeamMateData.numOfConnectedTeamMates > 2) {
    ledRequest.ledStates[LEDRequest::earsLeft180Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft216Deg] = LEDRequest::on;
  }
  if (theTeamMateData.numOfConnectedTeamMates > 3) {
    ledRequest.ledStates[LEDRequest::earsLeft252Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft288Deg] = LEDRequest::on;
  }
}

void LEDHandler::setEyeColor(LEDRequest& ledRequest, bool left, BehaviorLEDRequest::EyeColor col, LEDRequest::LEDState s) {
  LEDRequest::LED first = left ? LEDRequest::faceLeftRed0Deg : LEDRequest::faceRightRed0Deg;

  static const int redOffset = 0, greenOffset = LEDRequest::faceLeftGreen0Deg - LEDRequest::faceLeftRed0Deg,
                   blueOffset = LEDRequest::faceLeftBlue0Deg - LEDRequest::faceLeftRed0Deg,
                   numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg;

  LEDRequest::LEDState halfState = s == LEDRequest::off ? LEDRequest::off : LEDRequest::half;

  switch (col) {
  case BehaviorLEDRequest::defaultColor:
    ASSERT(false);
    break;
  case BehaviorLEDRequest::red:
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + redOffset + i] = s;
    }
    break;
  case BehaviorLEDRequest::green:
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + greenOffset + i] = s;
    }
    break;
  case BehaviorLEDRequest::blue:
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + blueOffset + i] = s;
    }
    break;
  case BehaviorLEDRequest::white:
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + redOffset + i] = s;
    }
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + greenOffset + i] = s;
    }
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + blueOffset + i] = s;
    }
    break;
  case BehaviorLEDRequest::magenta:
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + redOffset + i] = halfState;
    }
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + blueOffset + i] = s;
    }
    break;
  case BehaviorLEDRequest::yellow:
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + greenOffset + i] = halfState;
    }
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + redOffset + i] = s;
    }
    break;
  case BehaviorLEDRequest::cyan:
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + greenOffset + i] = halfState;
    }
    for (int i = 0; i <= numOfLEDsPerColor; i++) {
      ledRequest.ledStates[first + blueOffset + i] = s;
    }
    break;
  default:
    ASSERT(false);
    break;
  }
}

void LEDHandler::setLeftEye(LEDRequest& ledRequest) {
  // left eye -> groundContact ? ballSeen and GoalSeen : blue

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEye];

  // no groundContact
  if (!theGroundContactState.contact /* && (theFrameInfo.time & 512)*/) {
    setEyeColor(ledRequest, true, BehaviorLEDRequest::blue, state);
    // overwrite
  } else if (theBehaviorLEDRequest.leftEyeColor != BehaviorLEDRequest::defaultColor) {
    // blue
    setEyeColor(ledRequest, true, theBehaviorLEDRequest.leftEyeColor, state);
    // default
  } else {
    bool ballSeen = theBallModel.valid;
    bool combinedBallSeen = theCombinedWorldModel.ballIsValidOthers;

    if (thePersonalData.hasBallLock) {
      setEyeColor(ledRequest, true, BehaviorLEDRequest::red, state);
    } else {
      if (ballSeen && combinedBallSeen) {
        // red
        setEyeColor(ledRequest, true, BehaviorLEDRequest::green, state);
      } else if (ballSeen) {
        // white
        setEyeColor(ledRequest, true, BehaviorLEDRequest::white, state);
      } else if (combinedBallSeen) {
        // cyan
        setEyeColor(ledRequest, true, BehaviorLEDRequest::cyan, state);
      } else {
        // off
        setEyeColor(ledRequest, true, BehaviorLEDRequest::white, LEDRequest::off);
      }
    }
  }
  if (enableTeamMateDetectionLED) {
    applyTeamMateDetectionLEDAnimation(ledRequest);
  }
}

void LEDHandler::setRightEye(LEDRequest& ledRequest) {
  // right eye -> groundContact ? role : role -> blinking
  //           + penalty shootout: native_{striker,keeper} ? {striker,keeper} : off

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEye];

  // no groundContact
  if (!theGroundContactState.contact /* && (theFrameInfo.time & 512)*/) {
    setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
    // overwrite
  } else if (theBehaviorLEDRequest.rightEyeColor != BehaviorLEDRequest::defaultColor) {
    setEyeColor(ledRequest, false, theBehaviorLEDRequest.rightEyeColor, state);
  } else {
    switch (theBehaviorStatus.role) {
    case BehaviorStatus::keeper:
      setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
      break;
    case BehaviorStatus::defender:
      setEyeColor(ledRequest, false, BehaviorLEDRequest::green, state);
      break;
    case BehaviorStatus::striker:
      setEyeColor(ledRequest, false, BehaviorLEDRequest::red, state);
      break;
    default:
      setEyeColor(ledRequest, false, BehaviorLEDRequest::white, state);
      break;
    }
  }
  if (theRobotPose.lost && enableRobotLostLED) {
    applyEyeSwirlAnimation(false, ledRequest);
  }
}

void LEDHandler::applyEyeSwirlAnimation(bool left, LEDRequest& ledRequest) {
  if (left) {
    for (int i = 0; i < 6; ++i) {
      ledRequest.ledStates[LEDRequest::faceLeftGreen0Deg + (turningPositionCounter + i) % 8] = LEDRequest::off;
      ledRequest.ledStates[LEDRequest::faceLeftBlue0Deg + (turningPositionCounter + i) % 8] = LEDRequest::off;
      ledRequest.ledStates[LEDRequest::faceLeftRed0Deg + (turningPositionCounter + i) % 8] = LEDRequest::off;
    }
  } else {
    for (int i = 0; i < 6; ++i) {
      ledRequest.ledStates[LEDRequest::faceRightGreen0Deg + (turningPositionCounter + i) % 8] = LEDRequest::off;
      ledRequest.ledStates[LEDRequest::faceRightBlue0Deg + (turningPositionCounter + i) % 8] = LEDRequest::off;
      ledRequest.ledStates[LEDRequest::faceRightRed0Deg + (turningPositionCounter + i) % 8] = LEDRequest::off;
    }
  }
}

void LEDHandler::applyTeamMateDetectionLEDAnimation(LEDRequest& ledRequest) {
  for (int i = 0; i < 5; ++i) {
    ledRequest.ledStates[LEDRequest::faceLeftGreen90Deg + i] = LEDRequest::off;
    ledRequest.ledStates[LEDRequest::faceLeftBlue90Deg + i] = LEDRequest::off;
    ledRequest.ledStates[LEDRequest::faceLeftRed90Deg + i] = LEDRequest::off;
  }
  const Rangei validIndex(0, 5);
  for (auto player : thePlayerModel.players) {
    int ledIndex = static_cast<int>(-2.5f * std::atan2(player.relPosOnField.y, player.relPosOnField.x) / pi_4 + 2.5);
    ledIndex = validIndex.clamp(ledIndex);
    if (player.opponent) {
      ledRequest.ledStates[LEDRequest::faceLeftGreen90Deg + ledIndex] = LEDRequest::off;
      ledRequest.ledStates[LEDRequest::faceLeftBlue90Deg + ledIndex] = LEDRequest::off;
      ledRequest.ledStates[LEDRequest::faceLeftRed90Deg + ledIndex] = LEDRequest::on;
    } else {
      ledRequest.ledStates[LEDRequest::faceLeftGreen90Deg + ledIndex] = LEDRequest::off;
      ledRequest.ledStates[LEDRequest::faceLeftBlue90Deg + ledIndex] = LEDRequest::on;
      ledRequest.ledStates[LEDRequest::faceLeftRed90Deg + ledIndex] = LEDRequest::off;
    }
  }
}

void LEDHandler::setChestButton(LEDRequest& ledRequest) {
  if (theGameInfo.state == STATE_SET || theRobotInfo.penalty != PENALTY_NONE) {
    ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
  }
}

MAKE_MODULE(LEDHandler, Behavior Control)
