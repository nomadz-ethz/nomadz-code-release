/**
 * @file Motion.cpp
 *
 * Implementation of a class that represents the process that sends commands to the robot at 50Hz.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include "Motion.h"
#include "Modules/Infrastructure/NaoProvider.h"
#include "Modules/Infrastructure/MotionLogDataProvider.h"
#include "Modules/MotionControl/SpecialActions.h"
#include "Modules/MotionControl/MotionSelector.h"
#include "Core/System/Time.h"

#include <fenv.h>

static const char* categories[] = {"Motion Infrastructure", "Motion Control", "Sensing"};

Motion::Motion()
    : INIT_DEBUGGING, INIT_RECEIVER(CognitionToMotion), INIT_SENDER(MotionToCognition),
      moduleManager(categories, sizeof(categories) / sizeof(*categories)) {
  theDebugReceiver.setSize(200000);
  theDebugSender.setSize(40000);

  theMotionToCognitionSender.moduleManager = theCognitionToMotionReceiver.moduleManager = &moduleManager;
}

void Motion::init() {
  if (SystemCall::getMode() == SystemCall::physicalRobot) {
    setPriority(50);
  }

#ifdef ENABLE_ROS
  std::string node_name = "Motion_" + Global::getSettings().robot + "_" +
                          std::to_string(Global::getSettings().playerNumber) + "_" +
                          std::to_string(Global::getSettings().teamNumber);
  std::string node_namespace = Global::getSettings().robot + "_" + std::to_string(Global::getSettings().playerNumber) + "_" +
                               std::to_string(Global::getSettings().teamNumber);
  if (!rclcpp::ok()) {
    return;
  }
  auto ros_node = std::make_shared<rclcpp::Node>(node_name, node_namespace);
  moduleManager.load(ros_node);
#else
  moduleManager.load(nullptr);
#endif
  BH_TRACE_INIT("Motion");
}

void Motion::terminate() {
  moduleManager.destroy();
  Process::terminate();
}

bool Motion::main() {
  // there has been no new package from Cognition in more than 500ms and thus we let the robot stand.
  if (Time::getTimeSince(theCognitionToMotionReceiver.timeStamp) > 500) {
    MotionSelector::stand();
  }

  // required to detect whether any messages are sent in this frame
  int numberOfMessages = theDebugSender.getNumberOfMessages();

  if (MotionLogDataProvider::isFrameDataComplete() && NaoProvider::isFrameDataComplete()) {
    timingManager.signalProcessStart();

    STOP_TIME_ON_REQUEST_WITH_PLOT("Motion", moduleManager.execute(););
    NaoProvider::finishFrame();

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager", OUTPUT(idDrawingManager, bin, Global::getDrawingManager()););
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D",
                        OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D()););
    DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification",
                        OUTPUT(idStreamSpecification, bin, Global::getStreamHandler()););

    theMotionToCognitionSender.timeStamp = Time::getCurrentSystemTime();
    theMotionToCognitionSender.send();

    timingManager.signalProcessStop();
    DEBUG_RESPONSE("timing", timingManager.getData().copyAllMessages(theDebugSender););

    if (theDebugSender.getNumberOfMessages() != numberOfMessages) {
      // messages were sent in this frame -> send process finished
      OUTPUT(idProcessFinished, bin, 'm');
    }
    theDebugSender.send();
  }

  if (&Blackboard::theInstance->theJointData) {
    NaoProvider::waitForFrameData();
  } else {
    SystemCall::sleep(10);
  }

  return SystemCall::getMode() != SystemCall::physicalRobot;
}

bool Motion::handleMessage(InMessage& message) {
  switch (message.getMessageID()) {
  case idModuleRequest: {
    unsigned timeStamp;
    message.bin >> timeStamp;
    moduleManager.update(message.bin, timeStamp);
    return true;
  }

  default:
    return MotionLogDataProvider::handleMessage(message) || SpecialActions::handleMessage(message) ||
           Process::handleMessage(message);
  }
}

MAKE_PROCESS(Motion);
