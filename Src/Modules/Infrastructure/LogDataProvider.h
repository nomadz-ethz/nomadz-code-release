/**
 * @file LogDataProvider.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#pragma once

#include "Core/MessageQueue/MessageIDs.h"

#define UPDATE(representation) UPDATE2(representation, )
#define UPDATE2(representation, cmd)                                                                                        \
  void update(representation& _##representation) {                                                                          \
    if (representationBuffer[id##representation])                                                                           \
      _##representation = *((representation*)representationBuffer[id##representation]);                                     \
    cmd                                                                                                                     \
  }

#define ALLOC(representation)                                                                                               \
  if (!representationBuffer[id##representation])                                                                            \
    representationBuffer[id##representation] = new representation;

#define OUTREPRESENTATIONBUFFER(representation) *((representation*)representationBuffer[id##representation])
#define HANDLE(representation) HANDLE2(representation, )
#define HANDLE2(representation, cmd)                                                                                        \
  case id##representation: {                                                                                                \
    ALLOC(representation);                                                                                                  \
    message.bin >> OUTREPRESENTATIONBUFFER(representation);                                                                 \
    cmd return true;                                                                                                        \
  }

class LogDataProvider {
protected:
  Streamable* representationBuffer[numOfDataMessageIDs]; /**< The array of all logable representations. */

public:
  LogDataProvider() {
    for (int i = 0; i < numOfDataMessageIDs; i++) {
      representationBuffer[i] = 0;
    }
  }

  ~LogDataProvider() {
    for (int i = 0; i < numOfDataMessageIDs; i++) {
      if (representationBuffer[i]) {
        delete representationBuffer[i];
      }
    }
  }
};
