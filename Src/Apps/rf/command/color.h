#pragma once

#include <deque>
#include <iostream>
#include <map>
#include <string>

#include "command.h"

class ColorCmd : public CommandBase {

public:
  enum class ColorSpace { none, rgb, bgr, ycrcb };

  class Options {
  public:
    ColorSpace from;
    ColorSpace to;
    std::string source;
    std::string target;
    bool verbose;

    Options() : from(ColorSpace::none), to(ColorSpace::none), source(), target(), verbose(false) {}
  };

  static int main(const std::deque<std::string>&);

private:
  static const std::map<std::string, ColorSpace> colorSpaces;
  static const std::map<std::pair<ColorSpace, ColorSpace>, int> conversionCodes;

  static void printUsage();
  static bool parseOptions(const std::deque<std::string>&, Options&);

  static bool color(const Options&);
};
