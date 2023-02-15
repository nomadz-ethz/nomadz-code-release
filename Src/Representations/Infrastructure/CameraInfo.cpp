/**
 * @file CameraInfo.cpp
 *
 * Implementation of class CameraInfo
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#include "CameraInfo.h"

void CameraInfo::serialize(In* in, Out* out) {
  STREAM_REGISTER_BEGIN;
  STREAM(camera);
  STREAM(width);
  STREAM(height);
  STREAM(openingAngleWidth);
  STREAM(openingAngleHeight);
  STREAM(opticalCenter);
  if (in) {
    focalLength = width / (2.f * std::tan(openingAngleWidth / 2.f));
    focalLengthInv = 1.0f / focalLength;
    focalLenPow2 = focalLength * focalLength;
    focalLenPow4 = focalLenPow2 * focalLenPow2;
  }
  STREAM_REGISTER_FINISH;
}
