/**
 * @file RandomForests.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "RandomForests.h"

const LightForest* RandomForests::get(const std::string& name) const {
  auto it = forests.find(name);
  if (it != forests.end()) {
    return &(it->second);
  } else {
    return nullptr;
  }
}

void RandomForests::insert(const std::string& name, LightForest forest) {
  forests.insert(std::make_pair(name, std::move(forest)));
}
