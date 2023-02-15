/**
 * @file MotionLogDataProvider.cpp
 *
 * This file implements a module that provides data replayed from a log file.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "MotionLogDataProvider.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Settings.h"

PROCESS_WIDE_STORAGE(MotionLogDataProvider) MotionLogDataProvider::theInstance = 0;

#define ASSIGN(target, source)                                                                                              \
  ALLOC(target)                                                                                                             \
  (target&)*representationBuffer[id##target] = (target&)*representationBuffer[id##source];

MotionLogDataProvider::MotionLogDataProvider() : LogDataProvider(), frameDataComplete(false) {
  theInstance = this;
}

MotionLogDataProvider::~MotionLogDataProvider() {
  theInstance = 0;
}

bool MotionLogDataProvider::handleMessage(InMessage& message) {
  return theInstance && theInstance->handleMessage2(message);
}

bool MotionLogDataProvider::isFrameDataComplete() {
  if (!theInstance) {
    return true;
  } else if (theInstance->frameDataComplete) {
    OUTPUT(idLogResponse, bin, '\0');
    theInstance->frameDataComplete = false;
    return true;
  } else {
    return false;
  }
}

bool MotionLogDataProvider::handleMessage2(InMessage& message) {
  switch (message.getMessageID()) {
    HANDLE2(JointData, {
      ALLOC(FrameInfo);
      FrameInfo& frameInfo = (FrameInfo&)*representationBuffer[idFrameInfo];
      const JointData& jointData = (const JointData&)*representationBuffer[idJointData];
      frameInfo.cycleTime = (float)(jointData.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = jointData.timeStamp;
    })
    HANDLE(KeyStates)
    HANDLE(OdometryData)
    HANDLE(SensorData)
    HANDLE(FilteredSensorData)
    HANDLE(FilteredJointData)
    HANDLE(OrientationData)
    HANDLE2(GroundTruthOrientationData, ASSIGN(OrientationData, GroundTruthOrientationData))
    HANDLE2(GroundTruthOdometryData, ASSIGN(OdometryData, GroundTruthOdometryData))
    HANDLE(GameInfo) // but this is copied into and provided as RawGameInfo instead
    HANDLE(OwnTeamInfo)
    HANDLE(OpponentTeamInfo)
    HANDLE2(RobotInfo, ((RobotInfo&)*representationBuffer[idRobotInfo]).number = Global::getSettings().playerNumber;)

  case idProcessFinished:
    frameDataComplete = true;
    return true;

  default:
    return false;
  }
}

void MotionLogDataProvider::update(RawGameInfo& rawGameInfo) {
  if (representationBuffer[idGameInfo]) {
    rawGameInfo = *((RawGameInfo*)representationBuffer[idGameInfo]);
  }
}

void MotionLogDataProvider::update(GroundTruthOdometryData& groundTruthOdometryData) {
  if (representationBuffer[idGroundTruthOdometryData]) {
    groundTruthOdometryData = *((GroundTruthOdometryData*)representationBuffer[idGroundTruthOdometryData]);
  }
#ifndef RELEASE
  Pose2D odometryOffset(groundTruthOdometryData);
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionLogDataProvider:odometryOffsetX", odometryOffset.translation.x);
  PLOT("module:MotionLogDataProvider:odometryOffsetY", odometryOffset.translation.y);
  PLOT("module:MotionLogDataProvider:odometryOffsetRotation", toDegrees(odometryOffset.rotation));
  lastOdometryData = groundTruthOdometryData;
#endif
}

void MotionLogDataProvider::update(GroundTruthOrientationData& groundTruthOrientationData) {
  if (representationBuffer[idGroundTruthOrientationData]) {
    groundTruthOrientationData = *((GroundTruthOrientationData*)representationBuffer[idGroundTruthOrientationData]);
  }
#ifndef RELEASE
  PLOT("module:MotionLogDataProvider:gtOrientationX",
       toDegrees(atan2(groundTruthOrientationData.rotation.c1.z, groundTruthOrientationData.rotation.c2.z)));
  PLOT("module:MotionLogDataProvider:gtOrientationY",
       toDegrees(atan2(-groundTruthOrientationData.rotation.c0.z, groundTruthOrientationData.rotation.c2.z)));
  PLOT("module:MotionLogDataProvider:gtVelocityX", groundTruthOrientationData.velocity.x);
  PLOT("module:MotionLogDataProvider:gtVelocityY", groundTruthOrientationData.velocity.y);
  PLOT("module:MotionLogDataProvider:gtVelocityZ", groundTruthOrientationData.velocity.z);
#endif
}

MAKE_MODULE(MotionLogDataProvider, Motion Infrastructure)
