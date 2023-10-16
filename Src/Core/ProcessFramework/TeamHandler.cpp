/**
 * @file TeamHandler.cpp
 *
 * The file implements a class for team communication between robots.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TeamHandler.h"
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"
#include "Core/Global.h"
#include "Core/Settings.h"
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

  // TeamComm header
  // Byte | 0  1  2  3  |   5      6   |  7   8   |
  //      | time stamp  | payload size | robot id |

  const int bufferSize = 128; // Fixed maximum packet size
  const int teamCommHeaderSize = 8;

  char buffer[bufferSize];

  OutBinarySize sizeStream;
  sizeStream << out;
  const auto dataSize = sizeStream.getSize();

  int size = dataSize + teamCommHeaderSize;

  if (size > bufferSize) {
    OUTPUT_ERROR("TeamHandler could not send message because it was too big");
    out.clear();
    return;
  }

  OutBinaryMemory memory(buffer + teamCommHeaderSize);
  memory << out;

  *((uint32_t*)(buffer)) = Time::getCurrentSystemTime();
  *((uint16_t*)(buffer + 4)) = size;
  *((uint16_t*)(buffer + 6)) = Global::getSettings().playerNumber;

  if (socket.write(buffer, size)) {
    out.clear();
  }
}

void TeamHandler::receive() {
  if (!port) {
    return; // not started yet
  }

  const int bufferSize = 128;
  const int teamCommHeaderSize = 8;

  char buffer[bufferSize];

  int size;
  unsigned int remoteIp = 0;

  do {
    // TeamComm header
    // Byte | 0  1  2  3  |   5      6   |  7   8   |
    //      | time stamp  | payload size | robot id |
    size = socket.read(buffer, sizeof(buffer), remoteIp);
    if (size >= teamCommHeaderSize && *((uint16_t*)(buffer + 4)) == size) {
      unsigned int playerNum = *((uint16_t*)(buffer + 6));

      in.out.bin << (unsigned char)playerNum;     // the ID
      in.out.bin << *((uint32_t*)(buffer));       // the send time stamp
      in.out.bin << Time::getCurrentSystemTime(); // the receive time stamp
      in.out.finishMessage(idNTPHeader);

      in.out.bin << (int)playerNum;
      in.out.finishMessage(idRobot);

      InBinaryMemory memory(buffer + teamCommHeaderSize, size - teamCommHeaderSize);
      memory >> in;
      ASSERT(memory.eof());

      if (in.writeErrorOccurred()) {
        OUTPUT_TEXT("Write error for message received from " << static_cast<int>(playerNum));
      }
    }
  } while (size > 0);
}
