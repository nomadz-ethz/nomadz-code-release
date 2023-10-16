#include <deque>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#ifdef MULTI
#include <omp.h>
#endif
#include <opencv2/highgui/highgui.hpp>

#include "Tools/RandomForest/CostFunctions.h"
#include "Tools/RandomForest/RandomForest.h"
#include "Tools/RandomForest/SplitFunctions.h"
#include "Tools/RandomForest/StatisticFunctions.h"
#include "Tools/RandomForest/TreeConverter.h"

#include "train.h"
#include "util/common.h"

void TrainCmd::printUsage() {
  using std::cerr;
  using std::endl;

  cerr << "usage: rf train -i <class0-dir> [<class1-dir> ...] -o <forest-dir> [options]" << endl;
  cerr << endl;
  cerr << "options:" << endl;
  cerr << "  -a <patch-size> (default: 24)" << endl;
  cerr << "  -d <max-depth> (default: 10)" << endl;
  cerr << "  -l <min-samples-leaf> (default: 10)" << endl;
  cerr << "  -p <max-patches-per-class> (default: smallest number of patches)" << endl;
  cerr << "  -r <max-random-tests> (default: 1000)" << endl;
  cerr << "  -s <min-samples-split> (default: 20)" << endl;
  cerr << "  -t <num-trees> (default: 10)" << endl;
  cerr << "  -v" << endl;
}

bool TrainCmd::parseOptions(const std::deque<std::string>& args, Options& opts) {
  using std::cerr;
  using std::cout;
  using std::deque;
  using std::endl;
  using std::map;
  using std::string;
  using std::vector;

  // ez::ezOptionParser parser;
  // parser.add(    "", false, 0, '\0', "Display help", "-h", "--help");
  // parser.add(    "",  true, 1,  ' ', "Input directories (delimited by space)", "-i");
  // parser.add(    "",  true, 1, '\0', "Forest output directories", "-o");
  // parser.add(  "24", false, 1, '\0', "Patch size", "-a");
  // parser.add(  "10", false, 1, '\0', "Max depth", "-d");
  // parser.add(  "10", false, 1, '\0', "Min samples leaf", "-l");
  // parser.add(  "-1", false, 1, '\0', "Max patches per class (-1 is smallest number of patches", "-p");
  // parser.add("1000", false, 1, '\0', "Max random tests", "-r");
  // parser.add(  "20", false, 1, '\0', "Min samples split", "-s");
  // parser.add(  "10", false, 1, '\0', "Num trees", "-t");
  // parser.add(    "", false, 0, '\0', "Verbose output", "-v", "--verbose");

  // std::vector<char*> argv;
  // argv.push_back("color");
  // std::transform(args.begin(), args.end(), std::back_inserter(argv), [] (const string& s) {
  //   return s.c_str();
  // }]);

  // parser.parse(argv.size(), &argv[0]);

  // vector<string> missingArgs;
  // if (!parser.gotRequired(missingArgs))
  //   return false;

  // if (parser.isSet("-h"))
  //   return false;

  // parser.get("-i")->getStrings(opts.sources);
  // parser.get("-o")->getString(opts.target);

  // return false;

  // --help short-circuits everything else
  if (has(args, "-h") || has(args, "--help"))
    return false;

  if (args.size() < 2)
    return false;

  deque<string> todo = args;

  // <source-dir> ... <forest-dir>
  /*
  {
    vector<string> paths;
    while (todo.size() && todo.front()[0] != '-') {
      paths.push_back(cleanPath(todo.front()));
      todo.pop_front();
    }

    if (paths.size() < 2)
      return false;

    opts.target = paths.back();
    opts.sources = vector<string>(paths.begin(), paths.end() - 1);
  }
  */

  const map<string, bool&> boolFlags = {
    {"-v", opts.verbose},
    {"--verbose", opts.verbose},
  };

  const map<string, int&> intFields = {
    {"-a", opts.patchSize},
    {"-d", opts.maxDepth},
    {"-l", opts.minSamplesLeaf},
    {"-p", opts.maxPatchesPerClass},
    {"-r", opts.maxRandomTests},
    {"-s", opts.minSamplesSplit},
    {"-t", opts.numTrees},
  };

  const map<string, vector<string>&> stringVectorFields = {
    {"-i", opts.sources},
  };

  const map<string, string&> stringFields = {
    {"-o", opts.target},
  };

  while (todo.size()) {
    const string flag = todo.front();
    todo.pop_front();

    if (has(boolFlags, flag)) {
      boolFlags.at(flag) = true;

    } else if (has(intFields, flag)) {
      if (todo.size() < 1 || todo.front()[0] == '-')
        return false;

      try {
        intFields.at(flag) = std::stoi(todo.front());
      } catch (std::invalid_argument& e) {
        cerr << "rf: " << todo.front() << ": " << e.what() << endl;
      } catch (std::out_of_range& e) {
        cerr << "rf: " << todo.front() << ": " << e.what() << endl;
      }
      todo.pop_front();

    } else if (has(stringFields, flag)) {
      if (todo.size() < 1 || todo.front()[0] == '-')
        return false;

      stringFields.at(flag) = todo.front();
      todo.pop_front();

    } else if (has(stringVectorFields, flag)) {
      if (todo.size() < 1 || todo.front()[0] == '-')
        return false;

      vector<string>& stringVector = stringVectorFields.at(flag);
      while (todo.size() >= 1 && todo.front()[0] != '-') {
        if (has(todo.front(), ':')) {
          cerr << "whaaaa" << endl;
          return false;
        }
        stringVector.push_back(todo.front());
        todo.pop_front();
      }

    } else {
      cerr << "rf: unrecognized argument: " << flag << endl;
      return false;
    }
  }

  if (opts.sources.empty())
    return false;

  if (opts.target.empty())
    return false;

  const vector<std::reference_wrapper<vector<string>>> pathVectorFields = {opts.sources};
  for (auto i = pathVectorFields.begin(); i != pathVectorFields.end(); ++i)
    for (auto j = i->get().begin(); j != i->get().end(); ++j)
      *j = cleanPath(*j);

  return true;
}

int TrainCmd::main(const std::deque<std::string>& args) {
  using std::deque;
  using std::string;

  Options opts;
  if (!parseOptions(args, opts)) {
    printUsage();
    return 1;
  }

  return train(opts) ? 0 : 1;
}

bool TrainCmd::train(const Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;
  using std::string;
  using std::vector;

  cout << "sources: ";
  for (const auto& source : opts.sources) {
    cout << source << " ";
  }
  cout << endl;
  cout << "target: " << opts.target << endl;
  cout << "patch-size: " << opts.patchSize << endl;
  cout << "max-depth: " << opts.maxDepth << endl;
  cout << "min-samples-leaf: " << opts.minSamplesLeaf << endl;
  cout << "max-patches-per-class: " << opts.maxPatchesPerClass << endl;
  cout << "max-random-tests: " << opts.maxRandomTests << endl;
  cout << "min-samples-split: " << opts.minSamplesSplit << endl;
  cout << "num-trees: " << opts.numTrees << endl;
  cout << "verbose: " << opts.verbose << endl;

#ifdef MULTI
  cout << "num CPUs: " << omp_get_num_procs() << endl;
#endif

  const int numClasses = opts.sources.size();

  vector<int> labels;
  vector<string> fullPaths;
  for (int i = 0, n = numClasses; i < n; ++i) {
    const vector<string> classPaths = listFiles(opts.sources[i]);

    const vector<int> classLabels = vector<int>(classPaths.size(), i);
    labels.insert(labels.end(), classLabels.begin(), classLabels.end());

    fullPaths.insert(fullPaths.end(), classPaths.begin(), classPaths.end());
  }

  if (opts.verbose)
    cout << "reading images" << endl;

  vector<cv::Mat> mats;
  for (int i = 0, n = fullPaths.size(); i < n; ++i) {
    const string fullPath = fullPaths[i];

    cv::Mat mat;
    mat = cv::imread(fullPath);
    if (mat.data == NULL) {
      cerr << "rf: " << fullPath << ": cannot read as image" << endl;
      continue;
    }

    mats.push_back(mat);

    printProgress(i, n, 10000);
  }

  typedef RandomForest<cv::Mat, int, PixelDifference, LeafStatistic> MatClassifier;

#ifdef MULTI

#pragma omp parallel for
  for (int i = 0; i < opts.numTrees; ++i) {
    PixelDifference prototypeSplit(opts.patchSize);
    TreeBuilder<cv::Mat, PixelDifference, LeafStatistic, EntropyFunction> builder(
      prototypeSplit, numClasses, opts.maxDepth, opts.maxRandomTests, opts.minSamplesSplit, opts.minSamplesLeaf);
    MatClassifier forest = MatClassifier::train<EntropyFunction>(mats, labels, builder, 1);

    const std::string prefix = std::string("tree") + std::to_string(i);
    forest.saveXML(opts.target, prefix);

    // const RandomForest<LightPatch, int, LightPixelDifference, LeafStatistic> forest2 = TreeConverter::convert(forest);
    // forest2.saveBin(opts.target, prefix);
  }

#else
  PixelDifference prototypeSplit(opts.patchSize);
  TreeBuilder<cv::Mat, PixelDifference, LeafStatistic, EntropyFunction> builder(
    prototypeSplit, numClasses, opts.maxDepth, opts.maxRandomTests, opts.minSamplesSplit, opts.minSamplesLeaf);
  MatClassifier forest = MatClassifier::train<EntropyFunction>(mats, labels, builder, opts.numTrees);
  forest.saveXML(opts.target);
  const RandomForest<LightPatch, int, LightPixelDifference, LeafStatistic> forest2 = TreeConverter::convert(forest);
  forest2.saveBin(opts.target);
#endif

  return true;
}
