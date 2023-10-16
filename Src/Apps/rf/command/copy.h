#pragma once

#include <deque>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_set>

#include "command.h"

class CopyCmd : public CommandBase {

public:
  class Options {
  public:
    std::string files;

    int random;

    std::string similar;
    float similarThreshold;

    bool square;
    std::string source;
    std::string target;
    std::string others;
    bool verbose;

    Options()
        : files(), random(-1), similar(), similarThreshold(-1.f), square(false), source(), target(), others(),
          verbose(false) {}
  };

  static int main(const std::deque<std::string>&);

private:
  static void printUsage();
  static bool parseOptions(const std::deque<std::string>&, Options&);

  static std::unordered_set<std::string> pickSquare(const std::vector<std::string>&, const std::string&);
  static bool copy(const Options&);
};
