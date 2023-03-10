/**
 * @file BodyContour.h
 *
 * The file declares a class that represents the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "BodyContour.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "ColorReference.h"

BodyContour::BodyContourLine::BodyContourLine(const Vector2<int>& p1, const Vector2<int>& p2)
    : p1(p1.x < p2.x ? p1 : p2), p2(p1.x < p2.x ? p2 : p1) {}

void BodyContour::clipBottom(int x, int& y) const {
  int yIntersection;
  for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i) {
    if (i->yAt(x, yIntersection) && yIntersection < y) {
      y = yIntersection;
    }
  }
}
void BodyContour::clipBottom(int x, int& y, int imageHeight) const {
  clipBottom(x, y);

  // clippedY can be outside the image
  if (y < 0) {
    y = 0;
  } else if (y >= imageHeight) {
    y = imageHeight - 1;
  }
}

void BodyContour::clipLeft(int& x, int y) const {
  int xIntersection;
  for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i) {
    if (i->p1.y > i->p2.y) {
      if (i->xAt(y, xIntersection) && xIntersection > x) {
        x = xIntersection;
      } else if (i->p2.y <= y && i->p2.x > x) { // below a segment, clip anyway
        x = i->p2.x;
      }
    }
  }
}

void BodyContour::clipRight(int& x, int y) const {
  int xIntersection;
  for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i) {
    if (i->p1.y < i->p2.y) {
      if (i->xAt(y, xIntersection) && xIntersection < x) {
        x = xIntersection;
      } else if (i->p1.y <= y && i->p1.x < x) { // below a segment, clip anyway
        x = i->p1.x;
      }
    }
  }
}

void BodyContour::draw() const {
  DECLARE_DEBUG_DRAWING("representation:BodyContour", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BodyContour:maxY", "drawingOnImage");
  COMPLEX_DRAWING("representation:BodyContour", {
    for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i) {
      LINE("representation:BodyContour", i->p1.x, i->p1.y, i->p2.x, i->p2.y, 1, Drawings::ps_solid, ColorRGBA(255, 0, 255));
    }

    for (int x = 0; x < cameraResolution.x; x += 10) {
      int y = cameraResolution.y;
      clipBottom(x, y);
      LINE("representation:BodyContour", x, y, x, cameraResolution.y, 1, Drawings::ps_solid, ColorRGBA(255, 0, 255));
    }
  });
  int maxY = getMaxY();
  LINE(
    "representation:BodyContour:maxY", 0, maxY, cameraResolution.x - 1, maxY, 1, Drawings::ps_solid, ColorRGBA(255, 0, 255));
}

int BodyContour::getMaxY() const {
  int y = cameraResolution.y - 1;
  clipBottom(0, y);
  clipBottom(cameraResolution.x - 1, y);

  if (y < 0) { // no need to continue
    return 0;
  }

  for (const Line& line : lines) {
    if (line.p1.y >= 0 && line.p1.x >= 0 && line.p1.x < cameraResolution.x && line.p1.y < y) {
      y = line.p1.y;
    }
    if (line.p2.y >= 0 && line.p2.x >= 0 && line.p2.x < cameraResolution.x && line.p2.y < y) {
      y = line.p2.y;
    }
  }
  return y;
}
