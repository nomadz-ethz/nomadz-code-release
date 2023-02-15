/**
 * @file QueueFillRequest.cpp
 *
 * Implementation of the streaming operators for the QueueFillRequest.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href=mailto:dueffert@informatik.hu-berlin.de>Uwe Düffert</a>
 * and <a href=mailto:oberlies@sim.tu-darmstadt.de>Tobias Oberlies</a>
 */

#include "QueueFillRequest.h"

In& operator>>(In& stream, QueueFillRequest& queueFillRequest) {
  // The streamed data must be 8 bytes long (same as the previous version, to avoid corruption of the input queue)
  unsigned char temp;
  stream >> temp;
  queueFillRequest.behavior = static_cast<QueueFillRequest::Behavior>(temp);
  stream >> temp;
  queueFillRequest.filter = static_cast<QueueFillRequest::Filter>(temp);
  stream >> temp;
  queueFillRequest.target = static_cast<QueueFillRequest::Target>(temp);
  stream >> temp; // padding
  stream >> queueFillRequest.timingMilliseconds;
  return stream;
}

Out& operator<<(Out& stream, const QueueFillRequest& queueFillRequest) {
  // The streamed data must be 8 bytes long (by the way, a long is 4 bytes)
  stream << static_cast<unsigned char>(queueFillRequest.behavior);
  stream << static_cast<unsigned char>(queueFillRequest.filter);
  stream << static_cast<unsigned char>(queueFillRequest.target);
  stream << static_cast<unsigned char>(0);
  stream << queueFillRequest.timingMilliseconds;
  return stream;
}
