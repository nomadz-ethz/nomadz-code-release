/**
 * @file DebugHandler.cpp
 *
 * Class for debug communication over a TCP connection
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "DebugHandler.h"

#ifdef TARGET_ROBOT
#include "Core/System/BHAssert.h"
#include "Core/Global.h"
#include "Core/Settings.h"
#include "Core/Streams/OutStreams.h"
#include "Core/Streams/InStreams.h"

DebugHandler::DebugHandler(MessageQueue& in, MessageQueue& out, int maxPackageSendSize, int maxPackageReceiveSize)
    : TcpConnection(0, 0xA1BD, TcpConnection::receiver, maxPackageSendSize, maxPackageReceiveSize), in(in), out(out),
      sendData(0), sendSize(0) {}

void DebugHandler::communicate(bool send) {
  if (send && !sendData && !out.isEmpty()) {
    OutBinarySize size;
    size << out;
    sendSize = size.getSize();
    sendData = new unsigned char[sendSize];
    ASSERT(sendData);
    OutBinaryMemory memory(sendData);
    memory << out;
    out.clear();
  }

  unsigned char* receivedData;
  int receivedSize = 0;

  if (sendAndReceive(sendData, sendSize, receivedData, receivedSize) && sendSize) {
    delete[] sendData;
    sendData = 0;
    sendSize = 0;
  }

  if (receivedSize > 0) {
    InBinaryMemory memory(receivedData);
    memory >> in;
    delete[] receivedData;
  }
}
#endif