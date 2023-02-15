/**
 * @file ObservedFLIPMError.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 */

#pragma once

#ifndef WALKING_SIMULATOR
#include "Core/Streams/AutoStreamable.h"
#else
#include "bhumanstub.h"
#endif
#include "Core/Math/Matrix.h"
#include "Core/Math/Vector.h"
/**
 * @class ObservedFLIPMError
 *
 */
STREAMABLE_DECLARE_LEGACY(ObservedFLIPMError)
class ObservedFLIPMError : public ObservedFLIPMErrorBaseWrapper {

public:
  /** Constructor */
  ObservedFLIPMError() {
    for (int i = 0; i < 6; ++i)
      ObservedError[0][0][0][i] = 0;
    for (int i = 0; i < 6; ++i)
      ObservedError[0][1][0][i] = 0;
  };

  /** Destructor */
  ~ObservedFLIPMError(){};

  ObsError ObservedError;

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(ObservedError);
    STREAM_REGISTER_FINISH;
  };
};
