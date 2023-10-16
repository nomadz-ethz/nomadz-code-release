#include <deque>
#include <iomanip>
#include <iostream>
#include <string>

#include <opencv2/highgui/highgui.hpp>

#include "Tools/RandomForest/RandomForest.h"
#include "Tools/RandomForest/SplitFunctions.h"
#include "Tools/RandomForest/StatisticFunctions.h"

#include "predict.h"
#include "util/common.h"

void PredictCmd::printUsage() {
  using std::cout;
  using std::endl;

  cout << "usage: rf predict <forest-dir> <source-dir> <output-dir> [options]" << endl;
}

bool PredictCmd::parseOptions(const std::deque<std::string>& args, Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;

  if (has(args, "-h") || has(args, "--help"))
    return false;

  if (args.size() < 3)
    return false;

  opts.forest = cleanPath(args[0]);
  opts.source = cleanPath(args[1]);
  opts.output = cleanPath(args[2]);

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

int PredictCmd::main(const std::deque<std::string>& args) {
  using std::deque;
  using std::string;

  Options opts;
  if (!parseOptions(args, opts)) {
    printUsage();
    return 1;
  }

  return predict(opts) ? 0 : 1;
}

bool PredictCmd::predict(const Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;
  using std::string;
  using std::vector;

  cout << "forest: " << opts.forest << endl;
  cout << "source: " << opts.source << endl;
  cout << "output: " << opts.output << endl;
  cout << "verbose: " << opts.verbose << endl;

  const vector<string> forestPaths = listFiles(opts.forest);
  const vector<string> sourcePaths = listFiles(opts.source);

  typedef RandomForest<cv::Mat, int, PixelDifference, LeafStatistic> MatClassifier;
  MatClassifier forest = MatClassifier::loadXML(forestPaths);

  if (!isDir(opts.output))
    createDir(opts.output);

  for (int i = 0, n = sourcePaths.size(); i < n; ++i) {
    const string& sourcePath = sourcePaths[i];

    cv::Mat mat;
    mat = cv::imread(sourcePath);
    if (mat.data == NULL) {
      cerr << "rf: " << sourcePath << ": cannot be read as image" << endl;
      continue;
    }

    // std::map<int, float> probs = forest.predict(mat);
    // int label = MatClassifier::mostLikelyLabel(probs);
    // float tmpSum = 0.f;
    // cout << "[" << label << "]: ";
    // for (auto i = probs.begin(); i != probs.end(); ++i) {
    //   cout << " + (" << i->first << ", " << i->second << ")";
    //   tmpSum += i->second;
    // }
    // cout << " = " << tmpSum << endl;

    const std::map<int, float> probs = forest.predict(mat);
    const int label = MatClassifier::mostLikelyLabel(probs);

    const string outputDir = opts.output + "/" + std::to_string(label);
    if (!isDir(outputDir)) {
      createDir(outputDir);
    }

    const string outputPath = replaceDir(sourcePath, outputDir);
    copyFile(sourcePath, outputPath);

    if (opts.verbose) {
      cout << label << "\t";
      for (auto i = probs.begin(); i != probs.end(); ++i) {
        cout << std::setprecision(3) << i->second << "\t";
      }
      cout << basename(sourcePath) << endl;
    } else {
      printProgress(i, n, 10000);
    }
  }

  return true;
}
