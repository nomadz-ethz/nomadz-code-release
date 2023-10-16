/**
 * @file NaoProvider.cpp
 *
 * The file declares a module that provides information from the Nao via DCM.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include <cstdio>
#include <cstring>

#include "NaoProvider.h"

#ifdef TARGET_ROBOT

#include "naobridge/nomadz.h"

#ifdef MEASURE_DELAY
#include "Core/Streams/InStreams.h"
#endif
#include "Representations/Infrastructure/JointDataDeg.h"
#include "Core/System/Time.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Settings.h"

PROCESS_WIDE_STORAGE(NaoProvider) NaoProvider::theInstance = 0;

NaoProvider::NaoProvider() : gameControlTimeStamp(0) {
  NaoProvider::theInstance = this;

  OUTPUT(idText, text, "Hi, I am " << Global::getSettings().robot << ".");
  OUTPUT(idRobotname, bin, Global::getSettings().robot);

#ifndef RELEASE
  for (int i = 0; i < JointData::numOfJoints; ++i)
    clippedLastFrame[i] = JointData::off;
#endif
}

NaoProvider::~NaoProvider() {
  NaoProvider::theInstance = 0;
}

bool NaoProvider::isFrameDataComplete() {
  return true;
}

void NaoProvider::waitForFrameData() {
  if (theInstance)
    theInstance->naoBody.wait();
}

void NaoProvider::send() {
  DEBUG_RESPONSE("module:NaoProvider:lag100", SystemCall::sleep(100););
  DEBUG_RESPONSE("module:NaoProvider:lag200", SystemCall::sleep(200););
  DEBUG_RESPONSE("module:NaoProvider:lag300", SystemCall::sleep(300););
  DEBUG_RESPONSE("module:NaoProvider:lag1000", SystemCall::sleep(1000););
  DEBUG_RESPONSE("module:NaoProvider:lag3000", SystemCall::sleep(3000););
  DEBUG_RESPONSE("module:NaoProvider:lag6000", SystemCall::sleep(6000););
  DEBUG_RESPONSE("module:NaoProvider:segfault", *(volatile char*)0 = 0;);

  DEBUG_RESPONSE("module:NaoProvider:ClippingInfo", {
    for (int i = 0; i < JointData::numOfJoints; ++i) {
      if (i == JointData::RHipYawPitch) // missing on Nao
        ++i;

      if (theJointRequest.jointAngles.angles[i] != JointData::off) {
        if (theJointRequest.jointAngles.angles[i] > theJointCalibration.joints[i].maxAngle) {
          if (clippedLastFrame[i] != theJointCalibration.joints[i].maxAngle) {
            char tmp[64];
            sprintf(tmp,
                    "warning: clipped joint %s at %.03f, requested %.03f.",
                    JointData::getName((JointData::Joint)i),
                    toDegrees(theJointCalibration.joints[i].maxAngle),
                    toDegrees(theJointRequest.jointAngles.angles[i]));
            OUTPUT(idText, text, tmp);
            clippedLastFrame[i] = theJointCalibration.joints[i].maxAngle;
          }
        } else if (theJointRequest.jointAngles.angles[i] < theJointCalibration.joints[i].minAngle) {
          if (clippedLastFrame[i] != theJointCalibration.joints[i].minAngle) {
            char tmp[64];
            sprintf(tmp,
                    "warning: clipped joint %s at %.04f, requested %.03f.",
                    JointData::getName((JointData::Joint)i),
                    toDegrees(theJointCalibration.joints[i].minAngle),
                    toDegrees(theJointRequest.jointAngles.angles[i]));
            OUTPUT(idText, text, tmp);
            clippedLastFrame[i] = theJointCalibration.joints[i].minAngle;
          }
        } else
          clippedLastFrame[i] = JointData::off;
      }
    }
  });

#ifdef MEASURE_DELAY
  OutTextFile stream("delay.log", true);
  stream << "jointRequest";
  stream << theJointRequest.jointAngles.angles[JointData::LHipPitch];
  stream << theJointRequest.jointAngles.angles[JointData::LKneePitch];
  stream << theJointRequest.jointAngles.angles[JointData::LAnklePitch];
  stream << endl;
#endif

  float* actuators;
  naoBody.openActuators(actuators);
  int j = 0;
  ASSERT(headYawPositionActuator == 0);
  ASSERT(int(JointData::numOfJoints) - 1 == headYawHardnessActuator);

  for (int i = 0; i < JointData::numOfJoints; ++i) {
    if (i == JointData::RHipYawPitch) // missing on Nao
      ++i;

    if (theJointRequest.jointAngles.angles[i] == JointData::off) {
      actuators[j] = 0.0f;
      actuators[j + headYawHardnessActuator] = 0.0f; // hardness
    } else {
      actuators[j] = theJointRequest.jointAngles.angles[i] + theJointCalibration.joints[i].offset;
      actuators[j + headYawHardnessActuator] = float(theJointRequest.jointHardness.hardness[i]) / 100.f;
    }
    ++j;
  }
  j += lbhNumOfPositionActuatorIds;
  ASSERT(j == faceLedRedLeft0DegActuator);

  const LEDRequest& ledRequest(theLEDRequest);
  bool on = (theJointData.timeStamp / 50 & 8) != 0;
  bool fastOn = (theJointData.timeStamp / 10 & 8) != 0;
  for (int i = 0; i < LEDRequest::numOfLEDs; ++i)
    actuators[j++] = (ledRequest.ledStates[i] == LEDRequest::on || (ledRequest.ledStates[i] == LEDRequest::blinking && on) ||
                      (ledRequest.ledStates[i] == LEDRequest::fastBlinking && fastOn))
                       ? 1.0f
                       : (ledRequest.ledStates[i] == LEDRequest::half ? 0.5f : 0.0f);

  actuators[usActuator] = (float)theUSRequest.sendMode;

  // Time since last whistle in seconds
  float time_since_last_whistle = (theFrameInfo.time - theWhistle.lastTimeWhistleDetected) / 1000;

  naoBody.closeActuators();
  naoBody.setRobotInfo(
    Global::getSettings().teamNumber,
    Global::getSettings().teamColor,
    Global::getSettings().playerNumber,
    (theFallDownState.state == FallDownState::undefined || theFallDownState.state == FallDownState::upright) ? 0 : 1,
    theRobotPose.translation.x,
    theRobotPose.translation.y,
    normalize(theRobotPose.rotation),
    theBallModel.timeSinceLastSeen == INT_MAX ? -1.0f : theBallModel.timeSinceLastSeen / 1000.0f,
    theBallModel.estimate.position.x,
    theBallModel.estimate.position.y,
    theRefereePercept.gamecontrollerSignal,
    time_since_last_whistle);
}

void NaoProvider::update(JointData& jointData) {
  jointData.timeStamp = sensorData.timeStamp = std::max(jointData.timeStamp + 1, Time::getCurrentSystemTime());

  float* sensors = naoBody.getSensors();

  int j = 0;
  for (int i = 0; i < JointData::numOfJoints; ++i) {
    if (i == JointData::RHipYawPitch) {
      jointData.angles[i] = jointData.angles[JointData::LHipYawPitch];
      sensorData.currents[i] = sensorData.currents[JointData::LHipYawPitch];
      sensorData.temperatures[i] = sensorData.temperatures[JointData::LHipYawPitch];
    } else {
      jointData.angles[i] = sensors[j++] - theJointCalibration.joints[i].offset;
      sensorData.currents[i] = short(1000 * sensors[j++]);
      sensorData.temperatures[i] = (unsigned char)sensors[j++];
    }
  }

#ifdef MEASURE_DELAY
  OutTextFile stream("delay.log", true);
  stream << "timestamp" << Time::getCurrentSystemTime();
  stream << "jointData";
  stream << jointData.angles[JointData::LHipPitch];
  stream << jointData.angles[JointData::LKneePitch];
  stream << jointData.angles[JointData::LAnklePitch];
#endif

  for (int i = 0; i < SensorData::numOfSensors; ++i) {
    if (i >= SensorData::usL && i < SensorData::usREnd)
      ++j;
    else
      sensorData.data[i] = sensors[j++];
  }

  // sensorData.data[SensorData::gyroX] *= 12.f / 1600.f;
  // sensorData.data[SensorData::gyroY] *= 12.f / 1600.f;
  // sensorData.data[SensorData::gyroZ] *= 12.f / 1600.f;

  sensorData.data[SensorData::fsrLFL] =
    (sensorData.data[SensorData::fsrLFL] + theSensorCalibration.fsrLFLOffset) * theSensorCalibration.fsrLFLGain;
  sensorData.data[SensorData::fsrLFR] =
    (sensorData.data[SensorData::fsrLFR] + theSensorCalibration.fsrLFROffset) * theSensorCalibration.fsrLFRGain;
  sensorData.data[SensorData::fsrLBL] =
    (sensorData.data[SensorData::fsrLBL] + theSensorCalibration.fsrLBLOffset) * theSensorCalibration.fsrLBLGain;
  sensorData.data[SensorData::fsrLBR] =
    (sensorData.data[SensorData::fsrLBR] + theSensorCalibration.fsrLBROffset) * theSensorCalibration.fsrLBRGain;
  sensorData.data[SensorData::fsrRFL] =
    (sensorData.data[SensorData::fsrRFL] + theSensorCalibration.fsrRFLOffset) * theSensorCalibration.fsrRFLGain;
  sensorData.data[SensorData::fsrRFR] =
    (sensorData.data[SensorData::fsrRFR] + theSensorCalibration.fsrRFROffset) * theSensorCalibration.fsrRFRGain;
  sensorData.data[SensorData::fsrRBL] =
    (sensorData.data[SensorData::fsrRBL] + theSensorCalibration.fsrRBLOffset) * theSensorCalibration.fsrRBLGain;
  sensorData.data[SensorData::fsrRBR] =
    (sensorData.data[SensorData::fsrRBR] + theSensorCalibration.fsrRBROffset) * theSensorCalibration.fsrRBRGain;

#ifdef MEASURE_DELAY
  stream << "sensorData";
  stream << sensorData.data[SensorData::gyroX] << sensorData.data[SensorData::gyroY] << sensorData.data[SensorData::accX]
         << sensorData.data[SensorData::accY] << sensorData.data[SensorData::accZ];
#endif

  for (int i = 0; i < KeyStates::numOfKeys; ++i)
    keyStates.pressed[i] = sensors[j++] != 0;

  // ultasound

  if (theUSRequest.receiveMode != -1) {
    for (int i = SensorData::usL; i < SensorData::usREnd; ++i) {
      float data = sensors[i - SensorData::usL + lUsSensor];
      sensorData.data[i] = data != 0.f ? data * 1000.f : 2550.f;
    }
    sensorData.usTimeStamp = theJointData.timeStamp;
    sensorData.usActuatorMode = (SensorData::UsActuatorMode) static_cast<int>(theUSRequest.receiveMode);
  }

  PLOT("module:NaoProvider:usLeft", sensorData.data[SensorData::usL]);
  PLOT("module:NaoProvider:usRight", sensorData.data[SensorData::usR]);

  if (memcmp(&gameControlData, &naoBody.getGameControlData(), sizeof(RoboCup::RoboCupGameControlData))) {
    gameControlData = naoBody.getGameControlData();
    gameControlTimeStamp = theJointData.timeStamp;
  }

#ifndef RELEASE
  JointDataDeg jointDataDeg(jointData);
#endif
  MODIFY("representation:JointDataDeg", jointDataDeg);
}

void NaoProvider::update(SensorData& sensorData) {
  sensorData = this->sensorData;
  if (Time::getTimeSince(timeWhenBatteryLevelWritten) >= timeBetweenBatteryLevelUpdates) {
    timeWhenBatteryLevelWritten = Time::getCurrentSystemTime();
    OutTextFile(statusFilePath) << sensorData.data[SensorData::batteryLevel] << timeWhenBatteryLevelWritten;
  }
}

void NaoProvider::finishFrame() {
  if (theInstance)
    theInstance->send();
}

void NaoProvider::update(RobotInfo& robotInfo) {
  RoboCup::TeamInfo& team =
    gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  robotInfo.setFromRoboCupData(&team.players[Global::getSettings().playerNumber - 1]);
  robotInfo.number = Global::getSettings().playerNumber;
}

void NaoProvider::update(OwnTeamInfo& ownTeamInfo) {
  ownTeamInfo.setFromRoboCupData(
    &gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1]);
}

void NaoProvider::update(OpponentTeamInfo& opponentTeamInfo) {
  opponentTeamInfo.setFromRoboCupData(
    &gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 1 : 0]);
}

void NaoProvider::update(RawGameInfo& rawGameInfo) {
  rawGameInfo.setFromRoboCupData(&gameControlData);
  rawGameInfo.timeLastPackageReceived = gameControlTimeStamp;
}

#endif // TARGET_ROBOT

MAKE_MODULE(NaoProvider, Motion Infrastructure)
