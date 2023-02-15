/**
 * @file LibLost.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "../LibraryBase.h"

namespace behavior {
#include "LibLost.h"

  LibLost::LibLost() {
    bestCertainty = 0;
    timeLastLoc = 0;
    // bestPose = new Pose2D();
  }

  void LibLost::preProcess() {}
  void LibLost::postProcess() {}
} // namespace behavior
