/**
 * @file LightPatch.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Core/Math/Vector2.h"

/**
 * Describes a parallelogram-shaped patch in an image.
 * Only contains a reference to the image & a description of the shape within the image.
 */
class LightPatch {
public:
  // Reference to the image
  // (assumes image has 3 color channels);
  Image const& image;

  // Pixel coordinates of top-left corner
  Vector2<> origin;

  // Pixel offset of top-right corner from origin
  Vector2<> dir1;

  // Pixel offset of bottom-left corner from origin
  Vector2<> dir2;

  LightPatch(Image const& image, Vector2<> origin, Vector2<> dir1, Vector2<> dir2)
      : image(image), origin(origin), dir1(dir1), dir2(dir2) {}

  // Constructs from center & size of square
  static LightPatch createSquareFromCenter(Image const& image, Vector2<> c, float s) {
    return LightPatch(image, {c.x - s * 0.5f, c.y - s * 0.5f}, {s, 0.f}, {0.f, s});
  }

  // Constructs from sides of rectangle
  // patchSize: a compatibility hack for when patches don't use normalized coordinates ("patchSize" > 1)
  static LightPatch
  createRectFromSides(Image const& image, float xLeft, float xRight, float yTop, float yBottom, float patchSize = 1.f) {
    return LightPatch(image, {xLeft, yTop}, {(xRight - xLeft) / patchSize, 0}, {0, (yBottom - yTop) / patchSize});
  }

  inline bool xInside(int x) const { return x >= 0 && x < image.width; }

  inline bool yInside(int y) const { return y >= 0 && y < image.height; }

  // Return Image x-coord of (c1, c2)
  inline int xImg(float c1, float c2) const { return int(std::round(origin.x + c1 * dir1.x + c2 * dir2.x)); }

  // Return Image y-coord of (c1, c2)
  inline int yImg(float c1, float c2) const { return int(std::round(origin.y + c1 * dir1.y + c2 * dir2.y)); }

  // Takes two "parallelogram coordinates", each from 0 to 1, and returns the pixel at that location in the original image
  // (Uses nearest-neighbor spatial interpolation.)
  // Completes missing pixels by taking the mirror pixel within the patch, if that is available
  // TODO Export the resulting patches and check if it actually does this correctly
  inline Image::Pixel at(float c1, float c2) const {
    const int x = xImg(c1, c2);
    const int y = yImg(c1, c2);

    int x2, y2;

    if (xInside(x)) {
      if (yInside(y)) {
        return image[y][x];

      } else {
        // Above or below theImage
        x2 = xImg(c1, 1.f - c2);
        y2 = yImg(c1, 1.f - c2);
      }
    } else {
      if (yInside(y)) {
        // Left or right of theImage
        x2 = xImg(1.f - c1, c2);
        y2 = yImg(1.f - c1, c2);
      } else {
        // Outside of a corner of theImage
        x2 = xImg(1.f - c1, 1.f - c2);
        y2 = yImg(1.f - c1, 1.f - c2);
      }
    }

    if (xInside(x2) && yInside(y2)) {
      return image[y2][x2];

    } else {
      Image::Pixel blackPx;
      blackPx.color = 0x80008000;
      return blackPx;
    }
  }

  inline Image::Pixel at(const Vector2<>& pt) const { return this->at(pt.x, pt.y); }
};
