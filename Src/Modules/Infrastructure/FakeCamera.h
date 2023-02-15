/**
 * @file FakeCamera.h
 *
 * This file declares a module that provides fake camera images when the real image is not available (e.g. replaying certain
 * logs).
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/CameraMatrix.h"

MODULE(FakeCamera)
REQUIRES(CameraMatrix)
REQUIRES(CameraInfo)
REQUIRES(Image)
PROVIDES_WITH_OUTPUT(Image)
PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo)
END_MODULE

class FakeCamera : public FakeCameraBase {
private:
  float cycleTime;

  unsigned char* upperImage;
  unsigned char* lowerImage;

  void update(Image& image);
  void update(FrameInfo& frameInfo);

public:
  FakeCamera();
};
