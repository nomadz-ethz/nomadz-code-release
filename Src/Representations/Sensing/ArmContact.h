/**
 * @file ArmContact.h
 *
 * This file declares a class that represents if an ArmContact exists.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:dino.menges@tu-dortmund.de> Dino Menges</a>
 */

#pragma once

#ifndef WALKING_SIMULATOR
#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class ArmContact
 * A class that represents if an ArmContact exists.
 */
STREAMABLE_DECLARE_LEGACY(ArmContact)
class ArmContact : public ArmContactBaseWrapper {
protected:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(timeStampLeft);
    STREAM(timeStampRight);
    STREAM(armContactStateLeft);
    STREAM(armContactStateRight);
    STREAM_REGISTER_FINISH;
  }

public:
  ENUM(ArmContactState, None, Front, Back);

  /**
   * Default constructor.
   */
  ArmContact() : timeStampRight(0), timeStampLeft(0), armContactStateLeft(None), armContactStateRight(None) {}

  unsigned timeStampRight, timeStampLeft;
  ArmContactState armContactStateLeft, armContactStateRight;
};
