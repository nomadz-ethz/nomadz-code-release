/**
 * @file CameraInfo.h
 *
 * Declaration of class CameraInfo
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Enum.h"
#include "Core/Streams/AutoStreamable.h"

STREAMABLE_DECLARE_LEGACY(CameraInfo)

// TODO[repr]: Do not use manual streaming

/**
 * Information about the camera which provides the images for the robot
 */
class CameraInfo : public CameraInfoBaseWrapper {
public:
  /**
   * @enum Camera
   * Enum representing the possible sources of an image.
   */
  ENUM(Camera, upper, lower);

  Camera camera;
  int width;
  int height;
  float openingAngleWidth;
  float openingAngleHeight;
  Vector2<> opticalCenter;

  /** Intrinsic camera parameters: axis skew is modelled as 0 (90¬∞ perfectly orthogonal XY)
   * and the same has been modeled for focal axis aspect ratio; distortion is considering
   * only 2nd and 4th order coefficients of radial model, which account for about 95% of total.
   */
  float focalLength;
  float focalLengthInv; // (1/focalLength) used to speed up certain calculations
  float focalLenPow2;
  float focalLenPow4;

private:
  virtual void serialize(In* in, Out* out);
};
