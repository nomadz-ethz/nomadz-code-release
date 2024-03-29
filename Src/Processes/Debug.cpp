/**
 * @file Debug.cpp
 *
 * Implementation of class Debug.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Martin Lötzsch
 */

#include "Debug.h"
#include "Core/Debugging/Debugging.h"
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"

Debug::Debug()
    : INIT_EXTERNAL_DEBUGGING,

      INIT_DEBUG_RECEIVER(Cognition), INIT_DEBUG_RECEIVER(Motion),

      INIT_DEBUG_SENDER(Cognition), INIT_DEBUG_SENDER(Motion),

#ifdef ENABLE_ROS
      INIT_DEBUG_SENDER(ROSProcess), INIT_DEBUG_RECEIVER(ROSProcess),
#endif

      sendTime(0), processIdentifier(0), fout(0) {
  theDebugSender.setSize(MAX_PACKAGE_SEND_SIZE - 2000);
  theDebugReceiver.setSize(MAX_PACKAGE_RECEIVE_SIZE - 2000);
  theCognitionReceiver.setSize(10000000);
  theCognitionSender.setSize(1400000);

  theMotionReceiver.setSize(40000);
  theMotionSender.setSize(200000);

#ifdef ENABLE_ROS
  theROSProcessReceiver.setSize(40000);
  theROSProcessSender.setSize(200000);
#endif
}

bool Debug::main() {
  // Copying messages from debug queues from cognition and motion
  switch (outQueueMode.behavior) {
  case QueueFillRequest::sendCollected:
  case QueueFillRequest::discardNew:
  case QueueFillRequest::discardAll:
    // Discard new messages
    theCognitionReceiver.clear();
    theMotionReceiver.clear();
#ifdef ENABLE_ROS
    theROSProcessReceiver.clear();
#endif
    break;

  default:
    // Move the messages from other processes' debug queues to the outgoing queue
    if (!theCognitionReceiver.isEmpty()) {
      theCognitionReceiver.moveAllMessages(theDebugSender);
    }
#ifdef ENABLE_ROS
    // Replies from ROS are not distinguished (not needed)
    if (!theROSProcessReceiver.isEmpty()) {
      theROSProcessReceiver.moveAllMessages(theDebugSender);
    }
#endif
    if (!theMotionReceiver.isEmpty()) {
      OUTPUT(idProcessBegin, bin, 'm');
      theMotionReceiver.moveAllMessages(theDebugSender);
    }
  }

  // Handing behaviour
  bool sendNow = false, sendToGUI = false;
  switch (outQueueMode.behavior) {
  case QueueFillRequest::sendAfter:
    if (Time::getCurrentSystemTime() > sendTime) {
      // Send messages that are in the queue (now matter how long it takes), but don't take new messages
      sendNow = true;
      outQueueMode.behavior = QueueFillRequest::sendCollected;
    }
    break;

  case QueueFillRequest::sendEvery:
    if (Time::getCurrentSystemTime() > sendTime) {
      // Send now (if the network is busy, this send time is effectively skipped)
      sendNow = true;

      // Compute time for next sending
      sendTime = Time::getCurrentSystemTime() + outQueueMode.timingMilliseconds;
    }
    break;

  case QueueFillRequest::collect:
  case QueueFillRequest::discardNew:
    // Don't send now
    break;

  case QueueFillRequest::discardAll:
    // Clear output queue
    theDebugSender.clear();
    break;

  case QueueFillRequest::sendImmediately:
  case QueueFillRequest::sendCollected:
  default:
    sendNow = true;
  }

  if (sendNow) {
    // Apply filter
    switch (outQueueMode.filter) {
    case QueueFillRequest::latestOnly:
      // Send only latest of each type
      theDebugSender.removeRepetitions();
      break;

    case QueueFillRequest::sendEverything:; // Do nothing
    }

    // Send or save
    switch (outQueueMode.target) {
    case QueueFillRequest::writeToStick:
      if (!theDebugSender.isEmpty()) {
        if (!fout) {
          fout = new OutBinaryFile("logfile.log");
          theDebugSender.writeAppendableHeader(*fout);
        }
        // Append the outgoing queue to the file on the memory stick
        theDebugSender.append(*fout);
        theDebugSender.clear();
      }
      break;

    case QueueFillRequest::sendViaNetwork:
#ifdef TARGET_ROBOT
      if (messageWasReceived)
#endif
        sendToGUI = true;
      break;
    }
  }

// Send messages to the processes
#ifdef TARGET_SIM
  theCognitionSender.send(true);
  theMotionSender.send(true);
#ifdef ENABLE_ROS
  theROSProcessSender.send(true);
#endif
#else
  theCognitionSender.send(false);
  theMotionSender.send(false);
#ifdef ENABLE_ROS
  theROSProcessSender.send(false);
#endif
#endif

  DO_EXTERNAL_DEBUGGING(sendToGUI);
  return true;
}

void Debug::init() {
  if (SystemCall::getMode() == SystemCall::physicalRobot) {
    setPriority(5);
  }

  // read requests.dat
  InBinaryFile stream("requests.dat");
  if (stream.exists() && !stream.eof()) {
    stream >> theDebugReceiver;
  }

  theDebugReceiver.handleAllMessages(*this);
  theDebugReceiver.clear();
  messageWasReceived = false;
}

bool Debug::handleMessage(InMessage& message) {
  messageWasReceived = true;

  switch (message.getMessageID()) {
  case idText: // loop back to GUI
    message >> theDebugSender;
    return true;

  // messages to Cognition
  case idDescriptorCommand:
    message >> theCognitionSender;
    return true;

  // messages to Motion
  case idMotionNet:
  case idWalkingEngineKick:
    message >> theMotionSender;
    return true;

  // messages to Debug
  case idQueueFillRequest:
    // Read message queue settings and compute time when next to send (if in a timed mode)
    message.bin >> outQueueMode;
    sendTime = Time::getCurrentSystemTime() + outQueueMode.timingMilliseconds;
    if (fout) {
      delete fout;
      fout = 0;
    }
    return true;

  // messages to all except Debug (including ROS)
  case idModuleRequest:
  case idDebugDataChangeRequest:
    message >> theCognitionSender;
    message >> theMotionSender;
#ifdef ENABLE_ROS
    message >> theROSProcessSender;
#endif
    return true;

  // messages to all processes (including ROS)
  case idDebugRequest:
    message >> theCognitionSender;
    message >> theMotionSender;
#ifdef ENABLE_ROS
    message >> theROSProcessSender;
#endif
    return Process::handleMessage(message);

  case idProcessBegin:
    message.bin >> processIdentifier;
    message.resetReadPosition();
    // no break

  // message to either Motion or Cognition
  default:
    if (processIdentifier == 'm') {
      message >> theMotionSender;
    } else {
      message >> theCognitionSender;
    }

    return true;
  }
}

#ifndef RELEASE
MAKE_PROCESS(Debug);
#endif
