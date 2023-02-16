/**
 * @file ImageCoordinateSystem.cpp
 *
 * Implementation of a class that provides transformations on image coordinates.
 * Parts of this class were copied from class ImageInfo.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * and <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a>
 */

#include "ImageCoordinateSystem.h"
#include "Core/Debugging/DebugDrawings.h"

ImageCoordinateSystem::~ImageCoordinateSystem() {
  if (xTable) {
    delete[] xTable;
    delete[] yTable;
  }
}

void ImageCoordinateSystem::setCameraInfo(const CameraInfo& cameraInfo) {
  this->cameraInfo = cameraInfo;
  if (xTable) {
    return;
  }
  xTable = new int[maxResolutionWidth];
  yTable = new int[maxResolutionHeight];
  const float focalLength = cameraInfo.focalLength * maxResolutionWidth / cameraInfo.width;
  for (int i = 0; i < maxResolutionWidth; ++i) {
    xTable[i] = int(::atan((maxResolutionWidth / 2 - i) / focalLength) * 1024 + 0.5f);
  }
  for (int i = 0; i < maxResolutionHeight; ++i) {
    yTable[i] = int(::atan((i - maxResolutionHeight / 2) / focalLength) * 1024 + 0.5f);
  }
  for (int i = -3072; i < 3072; ++i) {
    table[i + 3072] = int(::tan(i / 1024.0f) * focalLength + 0.5f);
  }
}

void ImageCoordinateSystem::draw() const {
  DECLARE_DEBUG_DRAWING("horizon", "drawingOnImage"); // displays the horizon
  ARROW("horizon",
        origin.x,
        origin.y,
        origin.x + rotation[0].x * 50,
        origin.y + rotation[0].y * 50,
        0,
        Drawings::ps_solid,
        ColorRGBA(255, 0, 0));
  ARROW("horizon",
        origin.x,
        origin.y,
        origin.x + rotation[1].x * 50,
        origin.y + rotation[1].y * 50,
        0,
        Drawings::ps_solid,
        ColorRGBA(255, 0, 0));
}
