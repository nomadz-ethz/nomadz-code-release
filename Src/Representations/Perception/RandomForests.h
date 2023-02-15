/**
 * @file RandomForests.h
 *
 * Declaration of a representation containing the random forests for object evaluation
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <map>
#include <string>
#include "Core/Enum.h"
#include "Tools/RandomForest/RandomForest.h"
#include "Tools/RandomForest/SplitFunctions.h"
#include "Tools/RandomForest/StatisticFunctions.h"
#include "Core/Streams/AutoStreamable.h"

#include <opencv2/core/core.hpp>

STREAMABLE_DECLARE_LEGACY(RandomForests)
class RandomForests : public RandomForestsBaseWrapper {
public:
  ENUM(Label, negative, ball, robotBody, line);

  typedef std::vector<RandomForests::Label> LabelVector;
  typedef RandomForest<LightPatch, Label, LightPixelDifference, LeafStatistic> LightForest;

  const LightForest* get(const std::string& name) const;
  void insert(const std::string& name, LightForest forest);

private:
  std::map<std::string, LightForest> forests;

  virtual void serialize(In* in, Out* out) {}
};

// Export these typedefs (is this a good idea?)
typedef RandomForests::LightForest LightForest;
typedef RandomForests::Label RandomForestLabel;
