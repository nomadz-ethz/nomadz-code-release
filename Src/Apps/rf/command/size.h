#pragma once

#include <deque>
#include <iostream>
#include <string>

#include "command.h"

class SizeCmd : public CommandBase {

public:
  class Options {
  public:
    int size;
    std::string source;
    std::string target;
    bool verbose;

    Options() : size(-1), source(), target(), verbose(false) {}
  };

  static int main(const std::deque<std::string>&);

private:
  static void printUsage();
  static bool parseOptions(const std::deque<std::string>&, Options&);

  static bool size(const Options&);
};
