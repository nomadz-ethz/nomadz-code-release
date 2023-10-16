#pragma once

#include <deque>
#include <iostream>
#include <string>

#include "command.h"

class TrainCmd : public CommandBase {

public:
  class Options {
  public:
    std::vector<std::string> sources;
    std::string target;
    int patchSize;
    int maxDepth;
    int minSamplesLeaf;
    int maxPatchesPerClass;
    int maxRandomTests;
    int minSamplesSplit;
    int numTrees;
    bool verbose;

    Options()
        : sources(), target(), patchSize(24), maxDepth(10), minSamplesLeaf(10), maxPatchesPerClass(-1), maxRandomTests(1000),
          minSamplesSplit(20), numTrees(10), verbose(false) {}
  };

  static int main(const std::deque<std::string>&);

private:
  static void printUsage();
  static bool parseOptions(const std::deque<std::string>&, Options&);

  static bool train(const Options&);
};
