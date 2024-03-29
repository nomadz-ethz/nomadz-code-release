/**
 * @file ManualHeadMotionProvider.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include "ManualHeadMotionProvider.h"
#include "Tools/Geometry/Transformations.h"
#include "Core/Debugging/Debugging.h"

MAKE_MODULE(ManualHeadMotionProvider, Behavior Control)

ManualHeadMotionProvider::ManualHeadMotionProvider() : currentX(0), currentY(0) {}

void ManualHeadMotionProvider::update(HeadMotionRequest& headMotionRequest) {
  bool parametersChanged = xImg != currentX || yImg != currentY;
  if (parametersChanged && camera == theCameraInfo.camera) {
    currentX = xImg;
    currentY = yImg;

    headMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
    headMotionRequest.watchField = false;
    Vector2<float> targetOnField;
    Geometry::calculatePointOnField(currentX, currentY, theCameraMatrix, theCameraInfo, targetOnField);
    headMotionRequest.target.x = targetOnField.x;
    headMotionRequest.target.y = targetOnField.y;
    headMotionRequest.target.z = 0;

    // Use the camera that the user is seeing right now.
    switch (camera) {
    case CameraInfo::lower:
      headMotionRequest.cameraControlMode = HeadMotionRequest::lowerCamera;
      break;
    case CameraInfo::upper:
      headMotionRequest.cameraControlMode = HeadMotionRequest::upperCamera;
      break;
    default:
      ASSERT(false);
    }

    headMotionRequest.speed = fromDegrees(150);
  }
}
