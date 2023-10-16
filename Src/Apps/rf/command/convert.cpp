#include <deque>
#include <iostream>
#include <string>

#include "Tools/RandomForest/RandomForest.h"
#include "Tools/RandomForest/TreeConverter.h"
#include "Core/Global.h"

#include "convert.h"
#include "util/common.h"

using Format = ConvertCmd::Format;

const std::map<std::string, Format> formats = {
  {"mat", Format::mat},
  {"light", Format::light},
};

void ConvertCmd::printUsage() {
  using std::cerr;
  using std::endl;

  cerr << "usage: rf convert { mat | light } { mat | light } <source-forest> <target-forest> [-v | --verbose]" << endl;
}

bool ConvertCmd::parseOptions(const std::deque<std::string>& args, Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;

  if (has(args, "-h") || has(args, "--help"))
    return false;

  if (args.size() < 4)
    return false;

  if (!has(formats, args[0])) {
    cerr << "rf: '" << args[0] << "'' is not a valid random forest file format" << endl;
    return false;
  }
  opts.from = formats.at(args[0]);

  if (!has(formats, args[1])) {
    cerr << "rf: '" << args[1] << "'' is not a valid random forest file format" << endl;
    return false;
  }
  opts.to = formats.at(args[1]);

  opts.source = cleanPath(args[2]);
  opts.target = cleanPath(args[3]);

  // Process remaining flags
  for (auto i = args.cbegin() + 4; i != args.cend(); ++i) {
    if (*i == "-v" || *i == "--verbose") {
      opts.verbose = true;
    } else {
      cerr << "rf: unrecognized argument: " << *i << endl;
      return false;
    }
  }

  return true;
}

int ConvertCmd::main(const std::deque<std::string>& args) {
  using std::deque;
  using std::string;

  Options opts;
  if (!parseOptions(args, opts)) {
    printUsage();
    return 1;
  }

  return convert(opts) ? 0 : 1;
}

bool ConvertCmd::convert(const Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;
  using std::string;
  using std::vector;

  typedef RandomForest<cv::Mat, int, PixelDifference, LeafStatistic> MatRandomForest;
  typedef RandomForest<LightPatch, int, LightPixelDifference, LeafStatistic> LightRandomForest;

  cout << "source: " << opts.source << ", target: " << opts.target << endl;

  const vector<string> sourcePaths = listFiles(opts.source);

  if (opts.from == Format::mat) {
    MatRandomForest source;
    try {
      source = MatRandomForest::loadXML(sourcePaths);
    } catch (const cv::Exception& e) {
      source = MatRandomForest::loadBin(sourcePaths);
    }

    if (opts.to == Format::mat) {
      // mat -> mat
      source.saveXML(opts.target);
      source.saveBin(opts.target);

    } else if (opts.to == Format::light) {
      // mat -> light
      auto converted = TreeConverter::convert(source);
      converted.saveXML(opts.target);
      converted.saveBin(opts.target);
    }

  } else if (opts.from == Format::light) {
    LightRandomForest source;
    try {
      source = LightRandomForest::loadXML(sourcePaths);
    } catch (const cv::Exception& e) {
      source = LightRandomForest::loadBin(sourcePaths);
    }

    if (opts.to == Format::mat) {
      // light -> mat
      const int patchSize = 24;
      auto converted = TreeConverter::convert(source, patchSize);
      converted.saveXML(opts.target);
      converted.saveBin(opts.target);

    } else if (opts.to == Format::light) {
      // light -> light
      source.saveXML(opts.target);
      source.saveBin(opts.target);
    }
  }

  return true;
}