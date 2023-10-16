#pragma once

#include <deque>
#include <functional>
#include <string>

#include "Core/Streams/StreamHandler.h"
#include "Core/Global.h"

class Rf {
private:
  typedef std::function<int(const std::deque<std::string>&)> MainFunction;
  StreamHandler streamHandler;

  void printUsage(const std::deque<std::string>& args);
  bool parseCommand(const std::deque<std::string>& args, Rf::MainFunction& main, std::deque<std::string>& subargs);

public:
  Rf();

  int main(const std::deque<std::string>& args);
};
