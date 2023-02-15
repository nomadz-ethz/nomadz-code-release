/**
 * @file LibLost.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Math/Pose2D.h"

class LibLost : public LibraryBase {
public:
  LibLost();
  LibLost(internal::ref_init_tag t) : LibraryBase(t){};

  float bestCertainty{};
  Pose2D bestPose{};
  unsigned int timeLastLoc{};

  void preProcess() override;
  void postProcess() override;
};
