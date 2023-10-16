#include <assert.h>
#include <iostream>
#include <map>

#include "command/all.h"
#include "util/common.h"
#include "rf.h"

Rf::Rf() {
  // Enable streaming for all commands
  Global::theStreamHandler = &streamHandler;
}

void Rf::printUsage(const std::deque<std::string>& args) {
  using std::cout;
  using std::endl;

  cout << "usage: " << args[0] << " <command> [<args>]" << endl;
  cout << "       " << args[0] << " [<command>] [-h | --help]" << endl;
  cout << endl;
  cout << "random forest commands:" << endl;
  cout << "  convert  Convert a random forest between xml / bin formats" << endl;
  cout << "  predict  Classify images using a random forest" << endl;
  cout << "  train    Train a random forest" << endl;
  cout << "  validate Evaluate classification error" << endl;
  cout << endl;
  cout << "file handling commands:" << endl;
  cout << "  copy     Select certain files and copy them to another folder" << endl;
  cout << endl;
  cout << "image processing commands:" << endl;
  cout << "  color    Convert images from one color space to another" << endl;
  cout << "  size     Resize images to one fixed size" << endl;

#ifdef MULTI
  cout << endl;
  cout << "Compiled with multi-threading support." << endl;
#endif
}

bool Rf::parseCommand(const std::deque<std::string>& args, Rf::MainFunction& main, std::deque<std::string>& subargs) {

  const std::map<std::string, Rf::MainFunction> mains = {
    {"cl", ColorCmd::main},          {"col", ColorCmd::main},       {"colo", ColorCmd::main},
    {"color", ColorCmd::main},       {"cop", CopyCmd::main},        {"copy", CopyCmd::main},
    {"cp", CopyCmd::main},           {"cv", ConvertCmd::main},      {"con", ConvertCmd::main},
    {"conv", ConvertCmd::main},      {"conve", ConvertCmd::main},   {"conver", ConvertCmd::main},
    {"convert", ConvertCmd::main},   {"p", PredictCmd::main},       {"pr", PredictCmd::main},
    {"pre", PredictCmd::main},       {"pred", PredictCmd::main},    {"predi", PredictCmd::main},
    {"predic", PredictCmd::main},    {"predict", PredictCmd::main}, {"s", SizeCmd::main},
    {"si", SizeCmd::main},           {"siz", SizeCmd::main},        {"size", SizeCmd::main},
    {"t", TrainCmd::main},           {"tr", TrainCmd::main},        {"tra", TrainCmd::main},
    {"trai", TrainCmd::main},        {"train", TrainCmd::main},     {"v", ValidateCmd::main},
    {"va", ValidateCmd::main},       {"val", ValidateCmd::main},    {"vali", ValidateCmd::main},
    {"valid", ValidateCmd::main},    {"valida", ValidateCmd::main}, {"validat", ValidateCmd::main},
    {"validate", ValidateCmd::main},
  };

  if (args.size() >= 2 && has(mains, args[1])) {
    main = mains.at(args[1]);
    subargs.insert(subargs.end(), args.begin() + 2, args.end());
    return true;
  }

  return false;
}

int Rf::main(const std::deque<std::string>& args) {
  using std::cerr;
  using std::deque;
  using std::endl;
  using std::string;

  if (args.size() <= 1) {
    printUsage(args);
    return 1;
  }

  Rf::MainFunction cmd;
  deque<string> subargs;

  if (!parseCommand(args, cmd, subargs)) {
    if (has(args, "-h") || has(args, "--help") || has(args, "help"))
      printUsage(args);
    else
      cerr << "rf: '" << args[1] << "' is not a valid rf command. See 'rf --help'." << endl;
    return 1;
  }

  try {
    return cmd(subargs);
  } catch (std::exception& e) {
    cerr << "rf: " << e.what() << endl;
    return 1;
  } catch (...) {
    cerr << "rf: Uncaught exception" << endl;
    return 1;
  }
}

int main(int argc, const char** argv) {

  std::deque<std::string> args;
  for (int i = 0; i < argc; ++i) {
    args.push_back(argv[i]);
  }

  Rf rf;
  return rf.main(args);
}
