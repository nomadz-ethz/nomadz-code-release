/**
 * @file RandomForestsProvider.h
 *
 * This file declares a module which provides the random forest objects
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Perception/RandomForests.h"
#include "Representations/Perception/RandomForestsData.h"

MODULE(RandomForestsProvider)
PROVIDES(RandomForests)
LOADS_PARAMETER(std::string, pathToForests)
LOADS_PARAMETER(std::vector<RandomForestsLoad>, forests)
END_MODULE

class RandomForestsProvider : public RandomForestsProviderBase {
public:
  RandomForestsProvider();

  void update(RandomForests& randomForests);

private:
  bool loaded;

  std::vector<std::string> listPaths(const std::string& pathPattern);

  void loadForest(const RandomForestsLoad& forestLoad, RandomForests& randomForests);
};
