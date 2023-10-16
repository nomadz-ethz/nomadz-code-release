/**
 * @file CostFunctions.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <map>
#include "Tree.h"

// Describes what every cost function must have.
// This class is never really used anywhere as is, but is around for documentation (and helps the compiler catch mistakes).
class CostFunction {
public:
  virtual float operator()(const std::vector<int>&, const std::vector<int>&, size_t) = 0;
};

class EntropyFunction : public CostFunction {
public:
  // Operator for calculating entropy
  float operator()(const std::vector<int>& labels, const std::vector<int>& ids, size_t numberClasses) {
    if (ids.size() == 0)
      return 0;

    // For each class, stores number of patches of that class
    std::vector<int> numberPatches(numberClasses, 0);

    // Count the patches
    for (int i = 0, n = ids.size(); i < n; ++i)
      ++numberPatches[labels[ids[i]]];

    // Calculate the entropy
    float entropy = 0;
    for (int i = 0; i < (int)numberClasses; ++i)
      entropy += partialEntropy((float)numberPatches[i] / ids.size());

    return entropy;
  }

private:
  inline float partialEntropy(float p) { return (p < 0.00001 ? 0 : -p * (float)log(p)); }
};
