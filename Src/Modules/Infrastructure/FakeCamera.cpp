/**
 * @file FakeCamera.cpp
 *
 * This file declares a module that provides fake camera images when the real image is not available (e.g. replaying certain
 * logs).
 * The fake image is solid green below the horizon and solid gray above it.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <cstring>
#include <iostream>
#include "FakeCamera.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"

FakeCamera::FakeCamera() : cycleTime(1.f / 30.f), upperImage(nullptr), lowerImage(nullptr) {}

void FakeCamera::update(Image& image) {
  image.setResolution(theCameraInfo.width, theCameraInfo.height);

  const bool isUpper = (theCameraInfo.camera == CameraInfo::upper);
  const bool isLower = (theCameraInfo.camera == CameraInfo::lower);

  // Initialize images if necessary
  if (isUpper && !upperImage && image.widthStep * image.height > 0) {
    upperImage = (unsigned char*)malloc(sizeof(Image::Pixel) * image.widthStep * image.height);
  } else if (isLower && !lowerImage && image.widthStep * image.height > 0) {
    lowerImage = (unsigned char*)malloc(sizeof(Image::Pixel) * image.widthStep * image.height);
  }

  if ((isUpper && !upperImage) || (isLower && !lowerImage)) {
    std::cerr << "FakeCamera::update: could not allocate memory" << std::endl;
    return;
  }

  image.setImage(const_cast<unsigned char*>(isUpper ? upperImage : lowerImage));

  Geometry::Line horizon = Geometry::calculateHorizon(theCameraMatrix, theCameraInfo);
  Geometry::Line below = horizon;
  below.direction.rotateLeft();

  for (int y = 0, dy = -(int)(below.base.y); y < image.height; ++y, ++dy) {
    int dx = -(int)(below.base.x);
    for (Image::Pixel *p = image[y], *pEnd = p + image.width; p < pEnd; ++p, ++dx) {
      if (dx * below.direction.x + dy * below.direction.y > 0) {
        p->color = 0x555e5e00; // "grass"-green
      } else {
        p->color = 0x80808000; // "sky"-gray
      }
    }
  }

  image.timeStamp = Time::getRealSystemTime();
}

void FakeCamera::update(FrameInfo& frameInfo) {
  frameInfo.time = theImage.timeStamp;
  frameInfo.cycleTime = cycleTime * 0.5f;
}

MAKE_MODULE(FakeCamera, Cognition Infrastructure)
