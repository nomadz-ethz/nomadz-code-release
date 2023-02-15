/**
 * @file Pose2D.cpp
 *
 * Contains class Pose2D
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * and Max Risler
 */

#include "Pose2D.h"
#include "Core/Range.h"
#include "Core/Math/Random.h"

Pose2D Pose2D::random(const Range<float>& x, const Range<float>& y, const Range<float>& angle) {
  // angle should even work in wrap around case!
  return Pose2D(
    float(::randomFloat() * (angle.max - angle.min) + angle.min),
    Vector2<>(float(::randomFloat() * (x.max - x.min) + x.min), float(::randomFloat() * (y.max - y.min) + y.min)));
}
