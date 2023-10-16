#include <deque>
#include <iostream>
#include <string>

#include "validate.h"
#include "util/common.h"

void ValidateCmd::printUsage() {
  using std::cout;
  using std::endl;

  cout << "usage: rf validate <forest-dir> <source-dir>... [options]" << endl;
}

int ValidateCmd::main(const std::deque<std::string>& args) {
  using std::deque;
  using std::string;

  printUsage();
  return 1;
}
