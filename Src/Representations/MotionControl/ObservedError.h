/**
 * @file ObservedError.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#ifndef WALKING_SIMULATOR
#include "Core/Streams/AutoStreamable.h"
#else
#include "bhumanstub.h"
#endif
#include "Core/Math/Matrix.h"
/**
 * @class ObservedError
 *
 */
STREAMABLE_DECLARE_LEGACY(ObservedError)
class ObservedError : public ObservedErrorBaseWrapper {

public:
  /** Constructor */
  ObservedError(){};

  /** Destructor */
  ~ObservedError(){};

  Matrix<2, 1, Vector3f> CoM_WCS, ZMP_WCS;

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(CoM_WCS);
    STREAM(ZMP_WCS);
    STREAM_REGISTER_FINISH;
  };
};
