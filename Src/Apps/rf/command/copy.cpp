#include <deque>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <unordered_set>

#include <opencv2/highgui/highgui.hpp>

#include "copy.h"
#include "util/common.h"

void CopyCmd::printUsage() {
  using std::cerr;
  using std::endl;

  cerr << "usage: rf copy { --file <file-dir> | --random <count> | --similar <file-dir> <threshold> | --square } "
          "<source-dir> [[-t] <target-dir>] [-u <others-dir>] [-v]"
       << endl;
  cerr << "usage: rf copy --file <file-dir> <source-dir> [[-t] <target-dir>] [-u <others-dir>] [-v]" << endl;
  cerr << "usage: rf copy --random <count> <source-dir> [[-t] <target-dir>] [-u <others-dir>] [-v]" << endl;
  cerr << "usage: rf copy --similar <file-dir> <threshold> <source-dir> [[-t] <target-dir>] [-u <others-dir>] [-v]" << endl;
  cerr << "usage: rf copy --square <source-dir> [[-t] <target-dir>] [-u <others-dir>] [-v]" << endl;
}

// returns true iff syntax of input arguments are valid
bool CopyCmd::parseOptions(const std::deque<std::string>& args, Options& opts) {
  using std::cerr;
  using std::deque;
  using std::endl;
  using std::string;

  if (has(args, "-h") || has(args, "--help"))
    return false;

  deque<string> todo = args;

  // ... { [-f] <file-dir> | -r <count> | -s } ...
  if (todo.size() < 2)
    return false;

  // ... --file <file-dir> ...
  if (todo.at(0) == "--file") {

    if (todo.size() < 2)
      return false;

    opts.files = cleanPath(todo.at(1));

    todo.pop_front();
    todo.pop_front();

    // ... --random <count> ...
  } else if (todo.at(0) == "--random") {

    if (todo.size() < 2)
      return false;

    try {
      opts.random = std::stoi(todo.at(1));
    } catch (std::invalid_argument& e) {
      cerr << "rf: " << todo.at(1) << ": " << e.what() << endl;
      return false;
    } catch (std::out_of_range& e) {
      cerr << "rf: " << todo.at(1) << ": " << e.what() << endl;
      return false;
    }

    if (opts.random <= 0) {
      cerr << "rf: " << opts.random << ": must be a positive non-zero integer" << endl;
      return false;
    }

    todo.pop_front();
    todo.pop_front();

    // ... --similar <file-dir> <threshold>
  } else if (todo.at(0) == "--similar") {

    if (todo.size() < 3)
      return false;

    opts.similar = cleanPath(todo.at(1));

    try {
      opts.similarThreshold = std::stof(todo.at(2));
    } catch (std::invalid_argument& e) {
      cerr << "rf: " << todo.at(2) << ": " << e.what() << endl;
      return false;
    } catch (std::out_of_range& e) {
      cerr << "rf: " << todo.at(2) << ": " << e.what() << endl;
      return false;
    }

    if (opts.similarThreshold < 0) {
      cerr << "rf: " << opts.similarThreshold << ": must be a positive number" << endl;
      return false;
    }

    todo.pop_front();
    todo.pop_front();
    todo.pop_front();

    // ... --square ...
  } else if (todo.at(0) == "--square") {
    opts.square = true;
    todo.pop_front();
  }

  // ... <source-dir> ...
  {
    if (todo.size() < 1)
      return false;

    opts.source = cleanPath(todo.at(0));

    todo.pop_front();
  }

  // ... [[-t] <target-dir>] [-u <others-dir>] [-v | --verbose] ...
  while (!todo.empty()) {
    const string& flag = todo.at(0);

    // ... [-u <others-dir>] ...
    if (flag == "-u") {
      if (todo.size() < 1)
        return false;

      opts.others = cleanPath(todo.at(1));

      todo.pop_front();
      todo.pop_front();

      // ... [-v | --verbose] ...
    } else if (flag == "-v" || flag == "--verbose") {
      opts.verbose = true;
      todo.pop_front();

      // ... [[-t] <target-dir>] ...
    } else if (flag == "-t" || flag[0] != '-') {
      if (flag == "-t")
        todo.pop_front();

      if (todo.size() < 1)
        return false;

      opts.target = cleanPath(todo.at(0));
      todo.pop_front();

    } else {
      cerr << "rf: unrecognized argument: " << flag << endl;
      return false;
    }
  }

  return true;
}

int CopyCmd::main(const std::deque<std::string>& args) {
  using std::deque;
  using std::string;

  Options opts;
  if (!parseOptions(args, opts)) {
    printUsage();
    return 1;
  }

  return copy(opts) ? 0 : 1;
}

std::unordered_set<std::string> CopyCmd::pickSquare(const std::vector<std::string>& haystack, const std::string& base) {
  std::unordered_set<std::string> picked;
  for (int i = 0, n = haystack.size(); i < n; ++i) {
    const std::string fullPath = base + "/" + haystack[i];

    cv::Mat mat = cv::imread(fullPath);
    if (mat.data == NULL) {
      std::cerr << "rf: " << fullPath << ": cannot read as image" << std::endl;
      continue;
    }

    if (mat.rows == mat.cols) {
      picked.insert(haystack[i]);
    }

    printProgress(i, n, 10000);
  }

  return picked;
}

bool CopyCmd::copy(const Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;
  using std::set;
  using std::string;
  using std::unordered_set;
  using std::vector;

  cout << "files: " << opts.files << endl;
  cout << "random: " << opts.random << endl;
  cout << "similar: " << opts.similar << endl;
  cout << "square: " << opts.square << endl;
  cout << "source: " << opts.source << endl;
  cout << "target: " << opts.target << endl;
  cout << "others: " << opts.others << "\t(NOT IMPLEMENTED)" << endl;
  cout << "verbose: " << opts.verbose << endl;

  const vector<string> selectedPaths = (opts.files.empty() ? vector<string>() : listFiles(opts.files, false));

  const vector<string> sourcePaths = listFiles(opts.source, false);

  unordered_set<string> toCopy;
  if (!selectedPaths.empty()) {
    // Copy only sourcePaths that exist in selectedPaths
    cout << "rf: copy files in '" << opts.files << "' from '" << opts.source << "' to '" << opts.target << "'" << endl;

    set<string> selectedPathsSet(selectedPaths.begin(), selectedPaths.end());
    set<string> sourcePathsSet(sourcePaths.begin(), sourcePaths.end());
    std::set_intersection(selectedPathsSet.begin(),
                          selectedPathsSet.end(),
                          sourcePathsSet.begin(),
                          sourcePathsSet.end(),
                          std::inserter(toCopy, toCopy.begin()));

  } else if (opts.random > 0) {
    // Copy only a random subset of sourcePaths
    cout << "rf: copy " << opts.random << " random files from '" << opts.source << "' to '" << opts.target << "'" << endl;

    std::mt19937 gen;
    toCopy = pickSet(sourcePaths, opts.random, gen);

  } else if (opts.square) {
    // Copy only the square images in sourcePaths
    cout << "rf: copy square images from '" << opts.source << "' to '" << opts.target << "'" << endl;

    const string base = basename(opts.source);
    cout << "finding square images" << endl;
    toCopy = pickSquare(sourcePaths, base);
  }

  if (!opts.target.empty() && !isDir(opts.target)) {
    createDir(opts.target);
  }

  if (!opts.others.empty()) {
    cerr << "rf: ignoring -u: not yet implemented" << endl;
  }

  // if (!opts.others.empty() && !isDir(opts.others)) {
  //   createDir(opts.others);
  // }

  {
    const string sourceBase = basename(opts.source) + "/";
    const string targetBase = basename(opts.target) + "/";

    cout << "copying" << endl;

    const int numToCopy = toCopy.size();
    int i = 0;
    for (auto it = toCopy.begin(); it != toCopy.end(); ++it) {

      const string& filePath = *it;
      copyFile(sourceBase + filePath, targetBase + filePath);

      if (opts.verbose)
        cout << filePath << endl;
      else
        printProgress(i, numToCopy, 10000);

      ++i;
    }
  }

  return true;
}
