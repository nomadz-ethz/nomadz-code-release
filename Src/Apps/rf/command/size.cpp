#include <deque>
#include <iostream>
#include <string>

// #include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "size.h"
#include "util/common.h"

void SizeCmd::printUsage() {
  using std::cerr;
  using std::endl;

  cerr << "usage: rf size <new-size> <source-dir> <target-dir> [-v | --verbose]" << endl;
}

bool SizeCmd::parseOptions(const std::deque<std::string>& args, Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;

  if (has(args, "-h") || has(args, "--help"))
    return false;

  if (args.size() < 3)
    return false;

  try {
    opts.size = std::stoi(args[0]);

  } catch (std::invalid_argument& e) {
    cerr << "rf: " << args[0] << ": " << e.what() << endl;
    return false;

  } catch (std::out_of_range& e) {
    cerr << "rf: " << args[0] << ": " << e.what() << endl;
    return false;
  }

  if (opts.size <= 0) {
    cerr << "rf: " << opts.size << ": must be a positive non-zero number" << endl;
    return false;
  }

  opts.source = cleanPath(args[1]);
  opts.target = cleanPath(args[2]);

  // Process remaining flags
  for (auto i = args.cbegin() + 3; i != args.cend(); ++i) {
    if (*i == "-v" || *i == "--verbose") {
      opts.verbose = true;
    } else {
      cerr << "rf: unrecognized argument: " << *i << endl;
      return false;
    }
  }

  return true;
}

int SizeCmd::main(const std::deque<std::string>& args) {
  using std::deque;
  using std::string;

  Options opts;
  if (!parseOptions(args, opts)) {
    printUsage();
    return 1;
  }

  return size(opts) ? 0 : 1;
}

bool SizeCmd::size(const Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;

  cout << "source: " << opts.source << ", target: " << opts.target << ", new size: " << opts.size << endl;

  const std::vector<std::string> sourcePaths = listFiles(opts.source);

  if (!isDir(opts.target)) {
    createDir(opts.target);
  }

  for (int i = 0, n = sourcePaths.size(); i < n; ++i) {
    const std::string& sourcePath = sourcePaths[i];

    cv::Mat mat = cv::imread(sourcePath);
    if (mat.data == NULL) {
      cerr << "rf: " << sourcePath << ": imread failed" << endl;
      continue;
    }

    cv::resize(mat, mat, cv::Size(opts.size, opts.size), 0, 0, cv::INTER_CUBIC);

    const std::string targetPath = replaceDir(sourcePath, opts.target);

    try {
      cv::imwrite(targetPath, mat);
    } catch (cv::Exception& e) {
      cerr << "rf: " << targetPath << ": " << e.what() << endl;
    }

    if (opts.verbose)
      cout << targetPath << endl;
    else
      printProgress(i, n, 10000);
  }

  return true;
}