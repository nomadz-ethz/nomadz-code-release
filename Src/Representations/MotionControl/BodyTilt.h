/**
 * @file BodyTilt.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#pragma once
#ifndef WALKING_SIMULATOR
#include "Core/Streams/AutoStreamable.h"
#include "Core/Debugging/Watch.h"
#else
#include "bhumanstub.h"
#include "Watch.h"
#endif
#include "Tools/DortmundWalkingEngine/Point.h"

STREAMABLE_DECLARE_LEGACY(BodyTilt)
class BodyTilt : public BodyTiltBaseWrapper {
public:
  // angle around x and y
  float x, y;

  void watch() {
    WATCH(x);
    WATCH(y);
  }

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(x)
    STREAM(y)
    STREAM_REGISTER_FINISH;
  };
  BodyTilt() { x = y = 0; };

  ~BodyTilt(){};

  void operator=(const Point& p) {
    this->x = (float)p.x;
    this->y = (float)p.y;
  }
  void operator=(const BodyTilt& b) {
    this->x = (float)b.x;
    this->y = (float)b.y;
  }
};
