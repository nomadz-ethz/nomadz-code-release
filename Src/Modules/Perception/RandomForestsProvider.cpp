/**
 * @file RandomForestsProvider.cpp
 *
 * This file declares a module which provides the random forest objects
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "RandomForestsProvider.h"
#include "Core/System/Directory.h"
#include "Core/System/File.h"

#include <iostream>
#include <fstream>

MAKE_MODULE(RandomForestsProvider, Perception)

// Constructor
RandomForestsProvider::RandomForestsProvider() : loaded(false) {}

void RandomForestsProvider::update(RandomForests& randomForests) {
  if (loaded) {
    return;
  }

  // Load forests
  for (const RandomForestsLoad& loadData : forests) {
    loadForest(loadData, randomForests);
  }

  loaded = true;
}

std::vector<std::string> RandomForestsProvider::listPaths(const std::string& pathPattern) {
  Directory forestDir;
  forestDir.open(pathPattern);

  std::vector<std::string> forestPaths;
  std::string path;
  bool pathIsDir;
  while (forestDir.read(path, pathIsDir)) {
    if (!pathIsDir) {
      forestPaths.push_back(path);
    }
  }

  return forestPaths;
}

void RandomForestsProvider::loadForest(const RandomForestsLoad& forestLoad, RandomForests& randomForests) {
  // FIXME Only loading with LightForest::loadXML for now; should support loadBin in the future
  const std::string pathPattern =
    std::string(File::getBHDir()) + "/" + pathToForests + "/" + static_cast<std::string>(forestLoad.folder) + "/*.xml";
  const std::vector<std::string> forestPaths = listPaths(pathPattern);

  // Insert forest
  try {
    randomForests.insert(forestLoad.name, LightForest::loadXML(forestPaths, forestLoad.labels));
  } catch (std::runtime_error& e) {
    OUTPUT_WARNING("RandomForestsProvider::loadForest(" << static_cast<std::string>(forestLoad.name) << "): " << e.what());
  }
}
