/**
 * @file Footpositions.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#pragma once

#include "Tools/DortmundWalkingEngine/Point.h"
#include "Tools/DortmundWalkingEngine/StepData.h"

#ifndef WALKING_SIMULATOR
#include "Core/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class Robot
 * Representation to transfer the target foot (including the swing leg) positions to other modules.
 */
STREAMABLE_DECLARE_LEGACY(Footpositions)
class Footpositions : public Footposition, public FootpositionsBaseWrapper {
public:
  /** Is the controller running? */
  bool running;

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM_REGISTER_FINISH;
  };

  /** Constructor */
  Footpositions(){};

  /** Desctructor */
  ~Footpositions(){};

  Footpositions& operator=(const StepData& p) { return (Footpositions&)this->Footpositions::StepData::operator=(p); }

  Footposition& operator=(const Footposition& p) { return this->Footposition::operator=(p); }
};
