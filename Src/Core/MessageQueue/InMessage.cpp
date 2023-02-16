/**
 * @file InMessage.cpp
 *
 * Implementation of class InMessageQueue, InBinaryMessage, InTextMessage,
 * InConfigMessage and InMessage.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Martin Lötzsch
 */

#include "InMessage.h"
#include "Core/MessageQueue/MessageQueue.h"

InMessageQueue::InMessageQueue() : queue(0) {}

bool InMessageQueue::exists() const {
  return true;
}

bool InMessageQueue::getEof() const {
  return (queue != 0 ? queue->eof() : false);
}

void InMessageQueue::open(MessageQueueBase* q) {
  if (queue == 0) {
    queue = q;
  }
}

void InMessageQueue::readFromStream(void* p, int size) {
  if (queue != 0) {
    queue->read(p, size);
  }
}

InBinaryMessage::InBinaryMessage(MessageQueueBase* q) {
  open(q);
}

InTextMessage::InTextMessage(MessageQueueBase* q) {
  open(q);
}

std::string InTextMessage::readAll() {
  std::string result, s;
  while (!eof()) {
    *this >> s;
    result += s;
  }
  return result;
}

InConfigMessage::InConfigMessage(MessageQueueBase* q) {
  open(q);
}

InMessage::InMessage(MessageQueueBase& queue) : queue(queue), bin(&queue), text(&queue), config(&queue) {}

MessageID InMessage::getMessageID() const {
  return queue.getMessageID();
}

int InMessage::getMessageSize() const {
  return queue.getMessageSize();
}

int InMessage::getBytesLeft() const {
  return queue.getBytesLeftInMessage();
}

void InMessage::resetReadPosition() {
  queue.resetReadPosition();
  config.reset();
  text.reset();
}

const char* InMessage::getData() const {
  return queue.getData();
}
