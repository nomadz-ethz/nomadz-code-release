#pragma once

#include <deque>
#include <iostream>
#include <string>

#include "command.h"

class PredictCmd : public CommandBase {

public:
  class Options {
  public:
    std::string forest;
    std::string source;
    std::string output;
    bool verbose;

    Options() : forest(), source(), output(), verbose(false) {}
  };

  static int main(const std::deque<std::string>&);

private:
  static void printUsage();
  static bool parseOptions(const std::deque<std::string>&, Options&);

  static bool predict(const Options&);
};
