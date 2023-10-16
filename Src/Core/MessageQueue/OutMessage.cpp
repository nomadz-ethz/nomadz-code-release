/**
 * @file OutMessage.cpp
 *
 * Implementation of class OutMessage, OutBinaryMessage, OutTextMessage,
 * OutConfigMessage and OutMessageQueue.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Martin Lötzsch
 */

#include "OutMessage.h"
#include "Core/MessageQueue/MessageQueue.h"

OutMessageQueue::OutMessageQueue() : queue(0) {}

void OutMessageQueue::open(MessageQueueBase* q) {
  if (queue == 0) {
    queue = q;
  }
}

void OutMessageQueue::writeToStream(const void* p, int size) {
  if (queue != 0) {
    queue->write(p, size);
  }
}

OutBinaryMessage::OutBinaryMessage(MessageQueueBase* q) {
  open(q);
}

OutTextMessage::OutTextMessage(MessageQueueBase* q) {
  open(q);
}

OutTextRawMessage::OutTextRawMessage(MessageQueueBase* q) {
  open(q);
}

OutMessage::OutMessage(MessageQueueBase& queue) : queue(queue), bin(&queue), text(&queue), textRaw(&queue) {}

bool OutMessage::finishMessage(MessageID id) {
  return queue.finishMessage(id);
}

void OutMessage::cancelMessage() {
  queue.cancelMessage();
}
