/**
 * @file TeamHandler.cpp
 *
 * The file implements a class for team communication between robots.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TeamHandler.h"
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"
#include "Core/Debugging/Debugging.h"
#include "Core/Streams/OutStreams.h"
#include "Core/Streams/InStreams.h"

TeamHandler::TeamHandler(MessageQueue& in, MessageQueue& out) : in(in), out(out), port(0) {}

void TeamHandler::startLocal(int port) {
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(false));
  std::string group = SystemCall::getHostAddr();
  group = "239" + group.substr(group.find('.'));
  VERIFY(socket.bind("0.0.0.0", port));
  VERIFY(socket.setTTL(0)); // keep packets off the network. non-standard(?), may work.
  VERIFY(socket.joinMulticast(group.c_str()));
  VERIFY(socket.setTarget(group.c_str(), port));
}

void TeamHandler::start(int port, const char* subnet) {
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setTarget(subnet, port);
  socket.setLoopback(false);
}

void TeamHandler::send() {
  if (!port || out.isEmpty()) {
    return;
  }

  // TeamComm header (remote)
  // Byte |  0  1  2  3   |  4 .. x | x ...... x+4 |  x+5   x+6   | x+7  x+8 |
  //      |   "SPL "      | stdData |  time stamp  | payload size | robot id |

  const int bufferSize = sizeof(stdMsg);
  const int offsetofData = offsetof(RoboCup::SPLStandardMessage, data);
  const int teamCommHeaderSize = 6;

  char buffer[bufferSize];

  stdMsg = Global::getStdMsg();

  OutBinarySize sizeStream;
  sizeStream << out;
  const auto dataSize = sizeStream.getSize();

  stdMsg.numOfDataBytes = dataSize + teamCommHeaderSize;
  int size = offsetofData + dataSize + teamCommHeaderSize;

  if (size > sizeof(buffer)) {
    OUTPUT_ERROR("TeamHandler could not send message because it was too big");
    out.clear();
    return;
  }

  *((RoboCup::SPLStandardMessage*)buffer) = (RoboCup::SPLStandardMessage)stdMsg;

  OutBinaryMemory memory(buffer + offsetofData + teamCommHeaderSize);
  memory << out;

  *((uint32_t*)(buffer + offsetofData)) = Time::getCurrentSystemTime();
  *((uint16_t*)(buffer + offsetofData + 4)) = size;

  if (socket.write(buffer, size)) {
    out.clear();
  }
}

void TeamHandler::receive() {
  if (!port) {
    return; // not started yet
  }

  const int bufferSize = sizeof(stdMsg);
  const int offsetofData = offsetof(RoboCup::SPLStandardMessage, data);
  const int teamCommHeaderSize = 6;

  char buffer[bufferSize];

  int size;
  unsigned int remoteIp = 0;

  do {
    // TeamComm header
    // Byte |  0  1  2  3   |    4    5    |
    //      |   timestamp   | payload size |
    size = socket.read(buffer, sizeof(buffer), remoteIp);
    if (size >= (offsetofData + teamCommHeaderSize) && *((uint16_t*)(buffer + offsetofData + 4)) == size) {
      stdMsg = *((RoboCup::SPLStandardMessage*)buffer);

      in.out.bin << (unsigned char)stdMsg.playerNum;       // the ID
      in.out.bin << *((uint32_t*)(buffer + offsetofData)); // the send time stamp
      in.out.bin << Time::getCurrentSystemTime();          // the receive time stamp
      in.out.finishMessage(idNTPHeader);

      in.out.bin << (int)stdMsg.playerNum;
      in.out.finishMessage(idRobot);

      InBinaryMemory memory(buffer + offsetofData + teamCommHeaderSize, stdMsg.numOfDataBytes - teamCommHeaderSize);
      memory >> in;
      ASSERT(memory.eof());

      if (in.writeErrorOccurred()) {
        OUTPUT_TEXT("Write error for message received from " << static_cast<int>(stdMsg.playerNum));
      }
    }
  } while (size > 0);
}
