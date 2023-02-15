/**
 * @file ReferenceModificator.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "Tools/DortmundWalkingEngine/StepData.h"
#include "Tools/DortmundWalkingEngine/DynamicRingBuffer.h"

#ifndef WALKING_SIMULATOR
#include "Core/Streams/AutoStreamable.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class ReferenceModificator
 *
 */
STREAMABLE_DECLARE_LEGACY(ReferenceModificator)
struct ReferenceModificator : public ReferenceModificatorBaseWrapper {

  struct Time {
    int startZMP, startFoot[2];
  };
  /** Constructor */
  ReferenceModificator(){};

  /** Destructor */
  ~ReferenceModificator(){};

  /** Modificator in WCS for x modification in RCS */
  Point x;

  /** Modificator in WCS for y modification in RCS */
  Point y;

  /** The error that can be completely handeled by sidesteps */
  Matrix<2, 1, Vector3f> handledErr;

  const Point& operator[](int i) const {
    if (i == 0)
      return x;
    else
      return y;
  }

  Point& operator[](int i) {
    if (i == 0)
      return x;
    else
      return y;
  }

  union {
    struct {
      Time x, y;
    } time;
    Time aTime[2];
  };

  int creationTime;

private:
  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(x.x);
    STREAM(x.y);
    STREAM(y.x);
    STREAM(y.y);
    STREAM_REGISTER_FINISH;
  };
};
