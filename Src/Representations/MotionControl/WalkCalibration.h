/**
 * @file WalkCalibration.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */
#pragma once

#include "Tools/DortmundWalkingEngine/Point.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @class WalkCalibration
 *
 */
STREAMABLE_DECLARE_LEGACY(WalkCalibration)
struct WalkCalibration : public WalkCalibrationBaseWrapper {
  StreamPoint fixedOffset;
  bool calibrationDone;

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(fixedOffset)
    STREAM(calibrationDone)
    STREAM_REGISTER_FINISH;
  };

  /** Constructor */
  WalkCalibration(){};

  /** Desctructor */
  ~WalkCalibration(){};
};
