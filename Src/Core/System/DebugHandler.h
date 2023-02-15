/**
 * @file DebugHandler.h
 *
 * Inclusion of platform dependent code for debug communication.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:robocup@m-wachter.de">Michael Wachter</a>
 */

#pragma once

#ifdef TARGET_ROBOT

#include "Core/Debugging/TcpConnection.h"
#include "Core/MessageQueue/MessageQueue.h"

class DebugHandler : TcpConnection {
public:
  /**
   * Constructor.
   * @param in The message queue that stores data received.
   * @param out The message queue containing data to be sent.
   * @param maxPackageSendSize The maximum size of an outgoing package.
   *                           If 0, this setting is ignored.
   * @param maxPackageReceiveSize The maximum size of an incouming package.
   *                              If 0, this setting is ignored.
   */
  DebugHandler(MessageQueue& in, MessageQueue& out, int maxPackageSendSize = 0, int maxPackageReceiveSize = 0);

  /**
   * The method performs the communication.
   * It has to be called at the end of each frame.
   * @param send Send outgoing queue?
   */
  void communicate(bool send);

private:
  MessageQueue &in, /**< Incoming debug data is stored here. */
    &out;           /**< Outgoing debug data is stored here. */

  unsigned char* sendData; /**< The data to send next. */
  int sendSize;            /**< The size of the data to send next. */
};

#define EXTERNAL_DEBUGGING                                                                                                  \
  MessageQueue theDebugReceiver;                                                                                            \
  MessageQueue theDebugSender;                                                                                              \
  DebugHandler debugHandler;

#define INIT_EXTERNAL_DEBUGGING                                                                                             \
  Process(theDebugReceiver, theDebugSender), debugHandler(theDebugReceiver, theDebugSender, MAX_PACKAGE_SEND_SIZE, 0)

#define DO_EXTERNAL_DEBUGGING(s) debugHandler.communicate(s)

#endif

#ifdef TARGET_SIM

#define EXTERNAL_DEBUGGING DEBUGGING
#define INIT_EXTERNAL_DEBUGGING INIT_DEBUGGING
#define DO_EXTERNAL_DEBUGGING(s)                                                                                            \
  if (s)                                                                                                                    \
  theDebugSender.send()

#endif

#ifndef EXTERNAL_DEBUGGING
#error "Unknown platform or target"
#endif
