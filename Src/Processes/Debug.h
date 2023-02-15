/**
 * @file Debug.h
 *
 * Declaration of class Debug.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a> and Martin Lötzsch
 */

#pragma once

#define MAX_PACKAGE_SEND_SIZE 5000000
#define MAX_PACKAGE_RECEIVE_SIZE 1400000

#include "Core/ProcessFramework/Process.h"
#include "Core/System/DebugHandler.h"
#include "Core/Debugging/QueueFillRequest.h"

/**
 * @class Debug
 *
 * A process for collection and distribution of debug messages.
 *
 * All messages from the processes to the PC are collected here and all messages from the
 * PC to the processes are distributed by the Debug process.
 */
class Debug : public Process {
private:
  EXTERNAL_DEBUGGING;
  DEBUG_RECEIVER(Cognition);
  DEBUG_RECEIVER(Motion);
  DEBUG_SENDER(Cognition);
  DEBUG_SENDER(Motion);

#ifdef ENABLE_ROS
  DEBUG_SENDER(ROSProcess);
  DEBUG_RECEIVER(ROSProcess);
#endif

public:
  /** Constructor */
  Debug();

  /** Destructor */
  ~Debug() {
    if (fout)
      delete fout;
  }

  /** The main function of the process */
  bool main();

  /** Is called before the first main() */
  virtual void init();

  /**
   * Is called for every incoming debug message.
   * @param message the message to handle
   * @return if the message was handled
   */
  virtual bool handleMessage(InMessage& message);

  QueueFillRequest outQueueMode; /**< The mode (behavior, filter, target) for the outgoing queue. */
  unsigned sendTime;             /**< The next time the outgoing queue should be sent/written. */
  char processIdentifier;        /**< The process the messages from the GUI are meant to be sent to. */
  bool messageWasReceived;       /**< If true, a message was received from the WLan or OVirtualRobotComm. */
  OutBinaryFile* fout;           /**< Stream to store data on disk/stick. */
};
