/**
 * @file ManualHeadMotionProvider.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Core/Streams/Streamable.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(ManualHeadMotionProvider)
REQUIRES(CameraMatrix)
REQUIRES(CameraInfo)
PROVIDES_WITH_MODIFY(HeadMotionRequest)
DEFINES_PARAMETER(int, xImg, 0)
DEFINES_PARAMETER(int, yImg, 0)
DEFINES_PARAMETER(CameraInfo, Camera, camera, lower)
END_MODULE

class ManualHeadMotionProvider : public ManualHeadMotionProviderBase {
public:
  ManualHeadMotionProvider();

  /**
   * The update method to generate the head joint angles from desired head motion.
   */
  void update(HeadMotionRequest& headMotionRequest);

private:
  int currentX;
  int currentY;
};
