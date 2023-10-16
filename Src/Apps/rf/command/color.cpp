#include <deque>
#include <iostream>
#include <fstream>
#include <map>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "color.h"
#include "util/common.h"

using ColorSpace = ColorCmd::ColorSpace;

const std::map<std::string, ColorSpace> ColorCmd::colorSpaces = {
  {"rgb", ColorSpace::rgb},
  {"bgr", ColorSpace::bgr},
  {"ycrcb", ColorSpace::ycrcb},
};

const std::map<std::pair<ColorSpace, ColorSpace>, int> ColorCmd::conversionCodes = {
  {{ColorSpace::rgb, ColorSpace::bgr}, cv::COLOR_RGB2BGR},
  {{ColorSpace::bgr, ColorSpace::rgb}, cv::COLOR_BGR2RGB},
  {{ColorSpace::rgb, ColorSpace::ycrcb}, cv::COLOR_RGB2YCrCb},
  {{ColorSpace::ycrcb, ColorSpace::rgb}, cv::COLOR_YCrCb2RGB},
  {{ColorSpace::bgr, ColorSpace::ycrcb}, cv::COLOR_BGR2YCrCb},
  {{ColorSpace::ycrcb, ColorSpace::bgr}, cv::COLOR_YCrCb2BGR},
};

void ColorCmd::printUsage() {
  using std::cerr;
  using std::endl;

  cerr << "usage: rf color {rgb | bgr | ycrcb} {rgb | bgr | ycrcb} <source-dir> <target-dir> [-v]" << endl;
}

// returns true iff syntax of input arguments are valid
bool ColorCmd::parseOptions(const std::deque<std::string>& args, Options& opts) {
  using std::cerr;
  using std::endl;

  const auto& colorSpaces = ColorCmd::colorSpaces;
  const auto& conversionCodes = ColorCmd::conversionCodes;

  if (has(args, "-h") || has(args, "--help"))
    return false;

  if (args.size() < 4)
    return false;

  if (!has(colorSpaces, args[0])) {
    cerr << "rf: '" << args[0] << "' is not a valid color space" << endl;
    return false;
  }
  opts.from = colorSpaces.at(args[0]);

  if (!has(colorSpaces, args[1])) {
    cerr << "rf: '" << args[1] << "' is not a valid color space" << endl;
    return false;
  }
  opts.to = colorSpaces.at(args[1]);

  if (!has(conversionCodes, std::make_pair(opts.from, opts.to))) {
    cerr << "rf: conversion from '" << args[0] << "' to '" << args[1] << "' is not supported" << endl;
    return false;
  }

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

static bool hack(const ColorCmd::Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;
  using std::to_string;

  if (!isDir(opts.target)) {
    createDir(opts.target);
  }

  const std::vector<std::string> sourcePaths = listFiles(opts.source);
  for (int i = 0, n = sourcePaths.size(); i < n; ++i) {
    const std::string& sourcePath = sourcePaths[i];

    cv::Mat mat = cv::imread(sourcePath);
    if (mat.data == NULL) {
      cerr << sourcePath << ": cannot read as image" << endl;
      continue;
    }

    std::string name = std::regex_replace(basename(sourcePath), std::regex("\\.bmp"), std::string(""));
    std::stringstream ss(name);
    std::string item;
    std::vector<std::string> parts;
    while (std::getline(ss, item, '-')) {
      parts.push_back(item);
    }

    char cam = parts[4][0];
    int camW = (cam == 'U') ? 640 : 320;
    int camH = (cam == 'U') ? 480 : 240;
    cout << "camera\t" << cam << " (" << camW << " x " << camH << ")" << endl;
    int x = std::stoi(parts[5]);
    int y = std::stoi(parts[6]);
    int r = std::stoi(parts[7]);
    cout << "spot\t" << x << " " << y << " " << r << endl;

    int x0 = std::max(0, x - r);
    int y0 = std::max(0, y - r);
    int x1 = std::min(camW - 1, x + r);
    int y1 = std::min(camH - 1, y + r);
    cout << "patch\t(" << x0 << " " << y0 << ") (" << x1 << " " << y1 << ")" << endl;

    assert(mat.data);
    cout << "mat\t" << mat.cols << " x " << mat.rows << endl;

    cout << "want\t" << (2 * r) << " x " << (2 * r) << endl;
    cout << "expect\t" << (x1 - x0) << " x " << (y1 - y0) << endl;

    int dl = x0 - (x - r);
    int dr = x + r - (x1);
    int dt = y0 - (y - r);
    int db = y + r - (y1);

    std::string cuttedString = to_string(dl) + "-" + to_string(dr) + "-" + to_string(dt) + "-" + to_string(db);
    cout << "cutted\t" << cuttedString << endl;

    std::string targetPath =
      std::regex_replace(sourcePath, std::regex(to_string(x) + "-" + to_string(y) + "-" + to_string(r)), cuttedString);
    targetPath = replaceDir(targetPath, opts.target);
    cout << "save\t" << sourcePath << endl;
    cout << "to\t" << targetPath << endl;

    copyFile(sourcePath, targetPath);
  }

  return true;
}

int ColorCmd::main(const std::deque<std::string>& args) {
  Options opts;
  if (!parseOptions(args, opts)) {
    printUsage();
    return 1;
  }

  return color(opts) ? 0 : 1;
}

bool ColorCmd::color(const Options& opts) {
  using std::cerr;
  using std::cout;
  using std::endl;

  cout << "source: " << opts.source << ", target: " << opts.target << endl;

  // if (!isDir(opts.source)) {
  //   cerr << "rf: " << opts.source << " is not a directory" << endl;
  //   return false;
  // }
  const std::vector<std::string> sourcePaths = listFiles(opts.source);

  if (!isDir(opts.target)) {
    createDir(opts.target);
  }

  const int conversionCode = conversionCodes.at(std::make_pair(opts.from, opts.to));
  for (int i = 0, n = sourcePaths.size(); i < n; ++i) {
    const std::string& sourcePath = sourcePaths[i];

    cv::Mat mat = cv::imread(sourcePath);
    if (mat.data == NULL) {
      cerr << "rf: " << sourcePath << ": imread failed" << endl;
      continue;
    }

    cv::cvtColor(mat, mat, conversionCode);

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
