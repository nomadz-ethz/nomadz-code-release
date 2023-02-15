/**
 * @file ActualCoM.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "Tools/DortmundWalkingEngine/Point.h"

#ifndef WALKING_SIMULATOR
#include "Core/Streams/Streamable.h"
#include "Core/Debugging/Watch.h"
#else
#include "bhumanstub.h"
#include "Watch.h"
#endif

/**
 * @class ActualCoM
 * Representing the actual position of center of mass in the walking engines
 * world coordinate system.
 */

STREAMABLE_DECLARE_LEGACY(ActualCoM)
struct ActualCoM : public ActualCoMBaseWrapper, public Point {
  void watch() {
    WATCH(x);
    WATCH(y);
    WATCH(z);
  }

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(x)
    STREAM(y)
    STREAM(z)
    STREAM_REGISTER_FINISH;
  };

  /** Constructor */
  ActualCoM(){};

  /** Desctructor */
  ~ActualCoM(){};
};

struct ActualCoMRCS : public ActualCoM {};
struct ActualCoMFLIPM : public ActualCoM {};
