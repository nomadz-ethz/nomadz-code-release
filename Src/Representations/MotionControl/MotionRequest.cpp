/**
 * @file MotionRequest.cpp
 *
 * Implementation of a class that represents the motions that can be requested from the robot.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include <cstdio>
#include <cstring>

#include "MotionRequest.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"

void MotionRequest::printOut(char* destination) const {
  strcpy(destination, getName(motion));
  destination += strlen(destination);
  switch (motion) {
  case walk:
    if (walkRequest.mode == WalkRequest::targetMode) {
      sprintf(destination,
              ": %.0lfmm %.0lfmm %.0lf°",
              walkRequest.target.translation.x,
              walkRequest.target.translation.y,
              toDegrees(walkRequest.target.rotation));
    } else {
      sprintf(destination,
              ": %.0lfmm/s %.0lfmm/s %.0lf°/s",
              walkRequest.speed.translation.x,
              walkRequest.speed.translation.y,
              toDegrees(walkRequest.speed.rotation));
    }
    break;
  case specialAction:
    sprintf(destination, ": %s", SpecialActionRequest::getName(specialActionRequest.specialAction));
    break;
  case kick:
    sprintf(destination, ": %s", KickRequest::getName(kickRequest.kickMotionType));
    break;
  }
}

void MotionRequest::draw() const {
  DECLARE_DEBUG_DRAWING("representation:MotionRequest", "drawingOnField"); // drawing of a request walk vector
  if (motion == walk) {
    switch (walkRequest.mode) {
    case WalkRequest::targetMode: {
      LINE("representation:MotionRequest",
           0,
           0,
           walkRequest.target.translation.x,
           walkRequest.target.translation.y,
           0,
           Drawings::ps_solid,
           ColorRGBA(0xcd, 0, 0));
      CROSS("representation:MotionRequest",
            walkRequest.target.translation.x,
            walkRequest.target.translation.y,
            50,
            0,
            Drawings::ps_solid,
            ColorRGBA(0xcd, 0, 0));
      Vector2<> rotation(500.f, 0.f);
      rotation.rotate(walkRequest.target.rotation);
      ARROW("representation:MotionRequest",
            walkRequest.target.translation.x,
            walkRequest.target.translation.y,
            walkRequest.target.translation.x + rotation.x,
            walkRequest.target.translation.y + rotation.y,
            0,
            Drawings::ps_solid,
            ColorRGBA(0xcd, 0, 0, 127));
      break;
    }
    case WalkRequest::speedMode: {
      Vector2<> translation = walkRequest.mode == WalkRequest::speedMode ? walkRequest.speed.translation * 10.f
                                                                         : walkRequest.speed.translation * 1000.f;
      ARROW(
        "representation:MotionRequest", 0, 0, translation.x, translation.y, 0, Drawings::ps_solid, ColorRGBA(0xcd, 0, 0));
      if (walkRequest.target.rotation != 0.0f) {
        translation.x = translation.abs();
        translation.y = 0;
        translation.rotate(walkRequest.speed.rotation);
        ARROW("representation:MotionRequest",
              0,
              0,
              translation.x,
              translation.y,
              0,
              Drawings::ps_solid,
              ColorRGBA(0xcd, 0, 0, 127));
      }
      break;
    }
    }
  }
}

void MotionRequest::drawOnImage(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo) const {
  DECLARE_DEBUG_DRAWING("representation:MotionRequest:Image", "drawingOnImage");
  COMPLEX_DRAWING("representation:MotionRequest:Image", {
    if (motion == walk || true) {
      Vector2<int> meInImage;
      Geometry::calculatePointInImage(Vector2<>(0.f, 0.f), cameraMatrix, cameraInfo, meInImage);

      // Draw target, if applicable
      if (walkRequest.mode == WalkRequest::targetMode) {
        Vector2<int> targetInImage;
        if (Geometry::calculatePointInImage(walkRequest.target.translation, cameraMatrix, cameraInfo, targetInImage)) {
          LINE("representation:MotionRequest:Image",
               meInImage.x,
               meInImage.y,
               targetInImage.x,
               targetInImage.y,
               1,
               Drawings::ps_dash,
               ColorRGBA(0xcd, 0, 0));
          CROSS("representation:MotionRequest:Image",
                targetInImage.x,
                targetInImage.y,
                10,
                2,
                Drawings::ps_solid,
                ColorRGBA(0xcd, 0, 0));
        }

        std::vector<Vector2<>> spear = {{0.f, 0.f}, {1.f, 0.f}};
        const float spearScale = (cameraInfo.camera == CameraInfo::upper) ? 400.f : 200.f;
        for (Vector2<>& pt : spear) {
          pt.rotate(walkRequest.target.rotation);
          pt = pt * spearScale + walkRequest.target.translation;
        }

        std::vector<Vector2<int>> spearInImage;
        if (Geometry::calculatePointsInImage(spear, cameraMatrix, cameraInfo, spearInImage)) {
          ARROW("representation:MotionRequest:Image",
                spearInImage[0].x,
                spearInImage[0].y,
                spearInImage[1].x,
                spearInImage[1].y,
                1,
                Drawings::ps_solid,
                ColorRGBA(0xcd, 0, 0));
        }
      }

      // Draw speed if not zero
      if (walkRequest.speed != Pose2D(0.f, 0.f, 0.f)) {
        const Pose2D speedMax(0.7f, 100.f, 30.f);
        const float speedMaxBackwards = 50.f;

        Pose2D speed = walkRequest.speed;
        if (walkRequest.mode == WalkRequest::targetMode) {
          speed.rotation = std::max(-1.0f, std::min(speed.rotation, 1.0f));
          speed.translation.x = std::max(-1.0f, std::min(speed.translation.x, 1.0f));
          speed.translation.y = std::max(-1.0f, std::min(speed.translation.y, 1.0f));
          speed.rotation *= speedMax.rotation;
          speed.translation.x *= (speed.translation.x >= 0) ? speedMax.translation.x : speedMaxBackwards;
          speed.translation.y *= speedMax.translation.y;
        }

        const Vector2<int> drawingBaseInImage(cameraInfo.width / 2, cameraInfo.height * 7 / 8);
        Vector2<> drawingBase;
        const float drawingScale = (cameraInfo.camera == CameraInfo::upper) ? 2.f : 1.f;
        if (Geometry::calculatePointOnField(drawingBaseInImage, cameraMatrix, cameraInfo, drawingBase)) {
          std::vector<Vector2<>> axes = {{speed.translation.x, 0.f}, {0.f, speed.translation.y}};
          for (Vector2<>& pt : axes) {
            pt = pt * drawingScale + drawingBase;
          }

          std::vector<Vector2<int>> axesInImage;
          if (Geometry::calculatePointsInImage(axes, cameraMatrix, cameraInfo, axesInImage)) {

            // Axes
            ARROW("representation:MotionRequest:Image",
                  drawingBaseInImage.x,
                  drawingBaseInImage.y,
                  axesInImage[0].x,
                  axesInImage[0].y,
                  1,
                  Drawings::ps_solid,
                  ColorRGBA(0, 0, 0xcd));
            ARROW("representation:MotionRequest:Image",
                  drawingBaseInImage.x,
                  drawingBaseInImage.y,
                  axesInImage[1].x,
                  axesInImage[1].y,
                  1,
                  Drawings::ps_solid,
                  ColorRGBA(0, 0, 0xcd));

            // Text
            DRAWTEXT("representation:MotionRequest:Image",
                     drawingBaseInImage.x,
                     drawingBaseInImage.y,
                     10,
                     ColorRGBA(0xff, 0xff, 0xff),
                     std::round(speed.translation.abs()));
          }
        }
      }
    }
  });
}
