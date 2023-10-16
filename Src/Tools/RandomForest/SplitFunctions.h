/**
 * @file SplitFunctions.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <array>
#include <random>
#include <opencv2/core/core.hpp>
#include "LightPatch.h"
#include "Patch.h"
#include "Tree.h"
#include "Core/Math/Vector2.h"
#include "Core/Streams/Streamable.h"

// Describes what every split function must have.
// This class is never really used anywhere as is, but is around for documentation (and helps the compiler catch mistakes).
class SplitFunction {
public:
  // For documentation: you must implement these to use this class with Trees. (`virtual static` can't happen in C++ :/ )
  // virtual static SplitFunction load(const cv::FileNode&) = 0;

  virtual void randomize(std::mt19937&) = 0;

  virtual void save(cv::FileStorage&) const = 0;

  virtual bool equals(const SplitFunction&) const = 0;

  virtual bool operator==(const SplitFunction& other) const {
    if (typeid(*this) != typeid(other))
      return false;
    return this->equals(other);
  }

  // Depending on input type I:
  // virtual bool operator()(const I&) const = 0;
};

class LightPixelDifference;
class PixelDifference : public SplitFunction, public Streamable {

private:
  cv::Point pixel1, pixel2;
  int channel;

  friend class LightPixelDifference;
  friend class TreeConverter;

public:
  // Public parameters (for randomizing)
  unsigned int patchSize;

  // Also public (to help test how well this function fits things)
  float threshold;

  // Default constructor (that could also init public parameters)
  inline PixelDifference(unsigned int patchSize = 0) : patchSize(patchSize) {}

  // Convert from LightPixelDifference
  inline PixelDifference(const LightPixelDifference& other, int patchSize);

  // Called from TreeReadWrite class
  static PixelDifference load(const cv::FileNode& node) {
    PixelDifference f;

    node["Threshold"] >> f.threshold;
    node["Channel"] >> f.channel;

    // Workaround for older opencv library (not able to read as Point<> directly)
    std::vector<int> temp;
    node["Pixel1"] >> temp;
    f.pixel1 = cv::Point(temp[0], temp[1]);
    temp.clear();

    node["Pixel2"] >> temp;
    f.pixel2 = cv::Point(temp[0], temp[1]);

    return f;
  }

  virtual void serialize(In* in, Out* out) {
    int& x1 = pixel1.x;
    int& y1 = pixel1.y;
    int& x2 = pixel2.x;
    int& y2 = pixel2.y;

    STREAM_REGISTER_BEGIN;
    STREAM(x1);
    STREAM(y1);
    STREAM(x2);
    STREAM(y2);
    STREAM(channel);
    STREAM(patchSize);
    STREAM(threshold);
    STREAM_REGISTER_FINISH;
  }

  // Make split function random
  void randomize(std::mt19937& engine) {
    threshold = (float)(rand() % 256);

    channel = rand() % 3;

    pixel1 = cv::Point(rand() % patchSize, rand() % patchSize);
    pixel2 = cv::Point(rand() % patchSize, rand() % patchSize);
  }

  // Find threshold in [0 .. 255] that gives minimum cross-entropy for given data & labels
  float optimize(const std::vector<cv::Mat>& data, const std::vector<int>& labels, const std::vector<int>& selection) {
    // Find number of classes (-> among selected labels, the label with the highest value)
    int numClasses = 0;
    for (const int idx : selection)
      if (numClasses < labels[idx] + 1)
        numClasses = labels[idx] + 1;

    // int threshHistogram[int c][short t] = number of things of class "c" with thresh = "t"
    std::vector<std::array<int, 256>> threshHistogram(numClasses);
    for (int i = 0, n = selection.size(); i < n; ++i) {
      const int idx = selection[i];

      const int label = labels[idx];
      const cv::Mat& mat = data[idx];
      const short t = std::max<short>(0, mat.at<cv::Vec3b>(pixel1).val[channel] - mat.at<cv::Vec3b>(pixel2).val[channel]);

      ++threshHistogram[label][t];
    }

    // int leftCumulative[int c][short t] = number of things of class "c" with thresh < "t"
    std::vector<std::array<int, 256>> leftCumulative(numClasses);

    // int rightCumulative[int c][short t] = number of things of class "c" with thresh >= "t"
    std::vector<std::array<int, 256>> rightCumulative(numClasses);

    // Calculate cumulative counts
    for (int c = 0; c < numClasses; ++c) {
      auto& histogram = threshHistogram[c];
      auto& left = leftCumulative[c];
      auto& right = rightCumulative[c];

      left[0] = 0;
      for (short t = 0; t <= 255 - 1; ++t) {
        // left[1] = left[0] + histogram[0]
        //  ...
        // left[255] = left[254] + histogram[254]
        left[t + 1] = left[t] + histogram[t];
      }

      right[255] = histogram[255];
      for (short t = 255 - 1; t >= 0; --t) {
        // right[254] = right[255] + histogram[254]
        //  ...
        // right[0] = right[1] + histogram[0]
        right[t] = right[t + 1] + histogram[t];
      }
    }

    float bestCost = INFINITY;
    short bestThresh = 0;
    for (short t = 0; t <= 255; ++t) {
      int leftCount = 0, rightCount = 0;
      for (int c = 0; c < numClasses; ++c) {
        leftCount += leftCumulative[c][t];
        rightCount += rightCumulative[c][t];
      }

      if (leftCount == 0 || rightCount == 0) {
        // A pretty useless split
        continue;
      }

      float leftCost = 0.f, rightCost = 0.f;
      for (int c = 0; c < numClasses; ++c) {
        const float leftProb = leftCumulative[c][t] / (float)leftCount;
        leftCost += (leftProb < 1e-6) ? 0.f : -leftProb * std::log(leftProb);

        const float rightProb = rightCumulative[c][t] / (float)rightCount;
        rightCost += (rightProb < 1e-6) ? 0.f : -rightProb * std::log(rightProb);
      }

      const float cost = (leftCount * leftCost + rightCount * rightCost) / (float)(leftCount + rightCount);
      if (cost < bestCost) {
        bestCost = cost;
        bestThresh = t;
      }
    }

    threshold = (float)bestThresh;
    return bestCost;
  }

  // Method for saving tree to xml
  void save(cv::FileStorage& fs) const {
    fs << "Threshold" << threshold;
    fs << "Channel" << channel;
    fs << "Pixel1" << pixel1;
    fs << "Pixel2" << pixel2;
  }

  // Split operator
  bool operator()(const cv::Mat& mat) const {
    using cv::Vec3b;

    Vec3b intensity1 = mat.at<Vec3b>(pixel1);
    Vec3b intensity2 = mat.at<Vec3b>(pixel2);

    float val1 = intensity1.val[channel];
    float val2 = intensity2.val[channel];

    return val1 - val2 < threshold;
  }

  // Split operator for patch object
  inline bool operator()(const Patch& patch) const { return this->operator()(patch.mat); }

  bool equals(const SplitFunction& base) const {
    const PixelDifference& other = dynamic_cast<const PixelDifference&>(base);
    return pixel1 == other.pixel1 && pixel2 == other.pixel2 && channel == other.channel && patchSize == other.patchSize &&
           threshold == other.threshold;
  }
};

class LightPixelDifference : public SplitFunction, public Streamable {

private:
  Vector2<> pixel1, pixel2;
  short channel; // uses B-Human Image::Pixel convention! not OpenCV convention.

  friend class PixelDifference;

public:
  // Public (to help test how well this function fits things)
  short threshold;

  // Default constructor (that could also init public parameters)
  inline LightPixelDifference() {}

  // Convert from PixelDifference
  inline LightPixelDifference(const PixelDifference& other, int patchSize)
      : pixel1(float(other.pixel1.x) / patchSize, float(other.pixel1.y) / patchSize),
        pixel2(float(other.pixel2.x) / patchSize, float(other.pixel2.y) / patchSize), channel(short(other.channel)),
        threshold(short(other.threshold)) {}

  // Called from TreeReadWrite class
  static LightPixelDifference load(const cv::FileNode& node) {
    LightPixelDifference f;

    node["Threshold"] >> f.threshold;
    node["Channel"] >> f.channel;
    f.channel = cvToBhumanChannel(f.channel); // temporar; xml files currently use OpenCV convention

    // Workaround for older opencv library (not able to read as Point<> directly)
    std::vector<float> temp;
    node["Pixel1"] >> temp;
    f.pixel1 = Vector2<>(temp[0], temp[1]);
    temp.clear();

    node["Pixel2"] >> temp;
    f.pixel2 = Vector2<>(temp[0], temp[1]);

    return f;
  }

  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(pixel1);
    STREAM(pixel2);
    STREAM(channel);
    STREAM(threshold);
    STREAM_REGISTER_FINISH;
  }

  // Make split function random
  void randomize(std::mt19937& engine) {
    threshold = std::uniform_int_distribution<short>(0, 255)(engine);

    channel = std::uniform_int_distribution<char>(0, 2)(engine);

    std::uniform_real_distribution<float> reals(0.f, 1.f);
    pixel1 = Vector2<>(reals(engine), reals(engine));
    pixel2 = Vector2<>(reals(engine), reals(engine));
  }

  // Method for saving tree to xml
  void save(cv::FileStorage& fs) const {
    fs << "Threshold" << threshold;
    fs << "Channel" << bhumanToCvChannel(channel); // temporary; xml files currently use OpenCV convention
    fs << "Pixel1" << cv::Point2f(pixel1.x, pixel1.y);
    fs << "Pixel2" << cv::Point2f(pixel2.x, pixel2.y);
  }

  // Split operator
  inline bool operator()(const LightPatch& lightPatch) const {
    const float val1 = lightPatch.at(pixel1).channels[channel];
    const float val2 = lightPatch.at(pixel2).channels[channel];
    return val1 - val2 < threshold;
  }

  static short cvToBhumanChannel(short channel) {
    switch (channel) {
    case 0:
      return 2; // y
    case 1:
      return 3; // cr
    case 2:
      return 1; // cb
    default:
      return -1;
    }
  }

  static short bhumanToCvChannel(short channel) {
    switch (channel) {
    case 1:
      return 2; // cb
    case 2:
      return 0; // y
    case 3:
      return 1; // cr
    default:
      return -1;
    }
  }

  bool equals(const SplitFunction& base) const {
    const LightPixelDifference& other = dynamic_cast<const LightPixelDifference&>(base);
    return pixel1 == other.pixel1 && pixel2 == other.pixel2 && channel == other.channel && threshold == other.threshold;
  }
};

inline PixelDifference::PixelDifference(const LightPixelDifference& other, int patchSize)
    : pixel1(int(other.pixel1.x* patchSize), int(other.pixel1.y* patchSize)),
      pixel2(int(other.pixel2.x* patchSize), int(other.pixel2.y* patchSize)), channel(other.channel), patchSize(patchSize),
      threshold(other.threshold) {}