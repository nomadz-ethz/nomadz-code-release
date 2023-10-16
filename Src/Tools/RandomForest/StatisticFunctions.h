/**
 * @file StatisticFunctions.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <opencv2/core/core.hpp>

#include "Tree.h"
#include "Core/Streams/Streamable.h"

// Describes what every statistic function must have.
// This class is never really used anywhere as is, but is around for documentation (and helps the compiler catch mistakes).
class StatisticFunction {
public:
  // For documentation: you must implement these to use this class with Trees. (`virtual static` can't happen in C++ :/ )
  // virtual static StatisticFunction create(const std::vector<int>&, const std::vector<int>&, size_t) = 0;
  // virtual static StatisticFunction load(const cv::FileNode&) = 0;

  virtual void save(cv::FileStorage&) const = 0;

  virtual const std::vector<float>& probabilities() const = 0;

  virtual bool equals(const StatisticFunction&) const = 0;

  virtual bool operator==(const StatisticFunction& other) const {
    if (typeid(*this) != typeid(other))
      return false;
    return this->equals(other);
  }
};

class LeafStatistic : public StatisticFunction, public Streamable {
private:
  std::vector<float> classProbabilities; // anything that falls into this leaf node has these class probabilities
  int numPatches;                        // during training, how many patches went to this leaf node

protected:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(classProbabilities);
    STREAM(numPatches);
    STREAM_REGISTER_FINISH;
  }

public:
  // Called from TreeBuilder
  static LeafStatistic create(const std::vector<int>& labels, const std::vector<int>& patchIDs, size_t numberClasses) {
    LeafStatistic s;

    s.classProbabilities = std::vector<float>(numberClasses, 0.f);

    // Iterate over all patches of this leaf, calculate probabilities as mean frequency of appearance
    s.numPatches = patchIDs.size();
    for (int i = 0; i < s.numPatches; i++) {
      s.classProbabilities[labels[patchIDs[i]]] += 1.f / s.numPatches;
    }

    return s;
  }

  // Called from TreeReadWrite
  static LeafStatistic load(const cv::FileNode& node) {
    LeafStatistic s;

    size_t numberClasses = 0;
    for (cv::FileNodeIterator i = node.begin(); i != node.end(); ++i) {
      // Ugly, for historical reasons. We can't/shouldn't change the tree XML format anymore
      const cv::String& childName = (*i).name();
      if (childName[0] == 'P' && std::isdigit(childName[1]))
        ++numberClasses;
    }

    s.classProbabilities = std::vector<float>(numberClasses);
    for (size_t i = 0; i < numberClasses; i++) {
      node[("P" + std::to_string(i))] >> s.classProbabilities[i];
    }
    node["NumberElements"] >> s.numPatches;

    return s;
  }

  // Save method for saving tree to xml
  void save(cv::FileStorage& fs) const {
    // Iterate over all classes and write probabilities to xml
    for (unsigned int i = 0, n = classProbabilities.size(); i < n; i++) {
      std::string p = "P" + std::to_string(i);
      fs << p << classProbabilities[i];
    }
    // Write number of patches at this leaf to xml
    fs << "NumberElements" << numPatches;
  }

  inline const std::vector<float>& probabilities() const { return classProbabilities; }

  bool equals(const StatisticFunction& base) const {
    const LeafStatistic& other = dynamic_cast<const LeafStatistic&>(base);
    return classProbabilities == other.classProbabilities && numPatches == other.numPatches;
  }
};
