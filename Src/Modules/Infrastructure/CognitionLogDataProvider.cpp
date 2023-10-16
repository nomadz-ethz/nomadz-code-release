/**
 * @file CognitionLogDataProvider.cpp
 *
 * This file implements a module that provides data replayed from a log file.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "CognitionLogDataProvider.h"
#include "Core/Settings.h"
#include <vector>

PROCESS_WIDE_STORAGE(CognitionLogDataProvider) CognitionLogDataProvider::theInstance = 0;

#define ASSIGN(target, source)                                                                                              \
  ALLOC(target)                                                                                                             \
  (target&)*representationBuffer[id##target] = (target&)*representationBuffer[id##source];

CognitionLogDataProvider::CognitionLogDataProvider() : LogDataProvider(), frameDataComplete(false) {
  theInstance = this;
}

CognitionLogDataProvider::~CognitionLogDataProvider() {
  theInstance = 0;
}

bool CognitionLogDataProvider::handleMessage(InMessage& message) {
  return theInstance && theInstance->handleMessage2(message);
}

bool CognitionLogDataProvider::isFrameDataComplete() {
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

bool CognitionLogDataProvider::handleMessage2(InMessage& message) {
  switch (message.getMessageID()) {
    HANDLE2(Image, {
      ALLOC(FrameInfo)
      FrameInfo& frameInfo = (FrameInfo&)*representationBuffer[idFrameInfo];
      const Image& image = (const Image&)*representationBuffer[idImage];
      frameInfo.cycleTime = (float)(image.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = image.timeStamp;
    })
    HANDLE(CameraInfo)
    HANDLE(FrameInfo)
    HANDLE(LineAnalysis)
    HANDLE(ActivationGraph)
    HANDLE(BallPercept)
    HANDLE(GoalPercept)
    HANDLE(PenaltyMarkPercept)
    HANDLE(PlayerPercept)
    HANDLE(FieldBoundary)
    HANDLE(BallModel)
    HANDLE(BodyContour)
    HANDLE(Thumbnail)
    HANDLE(RobotHealth)
    HANDLE2(FilteredSensorData, {
      ALLOC(FrameInfo)
      FrameInfo& frameInfo = (FrameInfo&)*representationBuffer[idFrameInfo];
      const FilteredSensorData& filteredSensorData = (const FilteredSensorData&)*representationBuffer[idFilteredSensorData];
      frameInfo.cycleTime = (float)(filteredSensorData.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = filteredSensorData.timeStamp;
    })
    HANDLE2(BehaviorControlOutput,
            { behaviorControlOutput = (const BehaviorControlOutput&)*representationBuffer[idBehaviorControlOutput]; })
    HANDLE(FilteredJointData)
    HANDLE(PlayerModel)
    HANDLE(CombinedWorldModel)
    HANDLE(GroundTruthWorldState)
    HANDLE(CameraMatrix)

    HANDLE(ImageCoordinateSystem)
    HANDLE(RobotPose)
    HANDLE(RobotPoseSamples)
    HANDLE(SideConfidence)
    HANDLE(MotionInfo)
    HANDLE(ColorReference)
    HANDLE(TeamMateData)

  case idProcessFinished:
    frameDataComplete = true;
    return true;

  case idStopwatch: {
    const int size = message.getMessageSize();
    std::vector<unsigned char> data;
    data.resize(size);
    message.bin.read(&data[0], size);
    Global::getDebugOut().bin.write(&data[0], size);
    Global::getDebugOut().finishMessage(idStopwatch);
    return true;
  }
  case idJPEGImage:
    ALLOC(Image) {
      JPEGImage jpegImage;
      message.bin >> jpegImage;
      jpegImage.toImage((Image&)*representationBuffer[idImage]);
    }
    ALLOC(FrameInfo)
    ((FrameInfo&)*representationBuffer[idFrameInfo]).time = ((Image&)*representationBuffer[idImage]).timeStamp;
    return true;

  default:
    return false;
  }
}

MAKE_MODULE(CognitionLogDataProvider, Cognition Infrastructure)
