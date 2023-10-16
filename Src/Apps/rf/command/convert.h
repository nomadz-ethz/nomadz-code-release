#pragma once

#include <deque>
#include <iostream>
#include <string>

#include "command.h"

class ConvertCmd : public CommandBase {

public:
  enum class Format { none, mat, light };

  class Options {
  public:
    Format from;
    Format to;
    std::string source;
    std::string target;
    bool verbose;

    Options() : source(), target(), verbose(false) {}
  };

  static int main(const std::deque<std::string>&);

private:
  static void printUsage();
  static bool parseOptions(const std::deque<std::string>&, Options&);

  static bool convert(const Options&);
};
