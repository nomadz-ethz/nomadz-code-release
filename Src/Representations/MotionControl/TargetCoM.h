/**
 * @file TargetCoM.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
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
 * @class TargetCoM
 * Representing the target position of center of mass.
 */
STREAMABLE_DECLARE_LEGACY(TargetCoM)
class TargetCoM : public TargetCoMBaseWrapper, public Point, public Watch {
public:
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
    STREAM(state_x)
    STREAM(state_y)
    STREAM_REGISTER_FINISH;
  };

  float state_x[6], state_y[6];
  bool isRunning;

  /** Constructor */
  TargetCoM() : isRunning(false) {
    x = 0;
    y = 0;
    for (int i = 0; i < 6; i++) {
      state_x[i] = 0.0;
      state_y[i] = 0.0;
    }
  };

  /** Desctructor */
  ~TargetCoM(){};
};

// struct TargetCoMFLIPM : public TargetCoM {};
// struct TargetCoMLIPM : public TargetCoM {};
