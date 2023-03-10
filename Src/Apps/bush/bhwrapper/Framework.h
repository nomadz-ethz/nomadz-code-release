#pragma once

#include "Core/Settings.h"
#include "Core/MessageQueue/MessageQueue.h"
#include "Core/Debugging/DebugRequest.h"
#include "Core/Debugging/DebugDataTable.h"
#include "Core/Streams/StreamHandler.h"
#include <string>
#include <map>

/**
 * A wrapper for the BHuman framework. Create one instance and you do not have
 * to worry about any initializations. This is implemented as a singleton.
 */
class Framework {
  static std::map<std::string, Framework*> theInstances;

  Framework(const std::string& processName, int playerNumber, unsigned short port);
  Framework(const Framework& framework); /**< Should never be used. */
  Framework();                           /**< Should never be used. */
  ~Framework();

public:
  static Framework* getInstance(const std::string& processName, int playerNumber = 0, unsigned short port = 10101);
  static void destroy(const std::string& processName);

  MessageQueue debugOut;
  MessageQueue teamOut;
  Settings settings;
  DebugRequestTable debugRequestTable;
  DebugDataTable debugDataTable;
  StreamHandler streamHandler;
  int playerNumber;
};
