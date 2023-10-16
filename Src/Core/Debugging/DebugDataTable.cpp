/**
 * @file DebugDataTable.cpp
 *
 * This file implements a table for generic handling of streamable debug data.
 * Representations mentioned in the table will be overwritten with the table
 * entry.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Michael Spranger, Tobias Oberlies and Thomas Röfer
 */

#include "Core/Debugging/DebugDataTable.h"
#include "Core/MessageQueue/InMessage.h"

DebugDataTable::~DebugDataTable() {
  for (std::unordered_map<std::string, char*>::iterator iter = table.begin(); iter != table.end(); ++iter) {
    delete iter->second;
  }
}

void DebugDataTable::processChangeRequest(InMessage& in) {
  std::string name;
  char change;
  in.bin >> name >> change;
  std::unordered_map<std::string, char*>::iterator iter = table.find(name);
  if (change) {
    int size = in.getBytesLeft();
    char* buffer = new char[size];
    in.bin.read(buffer, size);
    if (iter == table.end()) {
      table[name] = buffer;
    } else {
      delete[] iter->second;
      iter->second = buffer;
    }
  } else if (iter != table.end()) {
    delete[] iter->second;
    table.erase(iter);
  }
}
