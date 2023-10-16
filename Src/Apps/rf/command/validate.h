#pragma once

#include <deque>
#include <iostream>
#include <string>

#include "command.h"

class ValidateCmd : public CommandBase {

private:
  static void printUsage();

public:
  static int main(const std::deque<std::string>&);
};
