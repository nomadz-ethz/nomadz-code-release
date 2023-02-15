/**
 * @file ZMPModel.h
 *
 * Declaration of class ZMPModel.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
 */

#pragma once
#ifndef WALKING_SIMULATOR
#include "Core/Streams/AutoStreamable.h"
#include "Core/Debugging/Watch.h"
#endif
#include "Core/Math/Vector.h"
#include "Core/Math/Vector2.h"
#include "Core/Math/Vector3.h"

/**
 * @class ZMPModel
 *
 * Contains information about ZMP.
 */
STREAMABLE_DECLARE_LEGACY(ZMPModel)
class ZMPModel : public ZMPModelBaseWrapper {
  /** Streaming */
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(zmp_acc);
    STREAM(ZMP_WCS);
    STREAM_REGISTER_FINISH;
  }

public:
  Vector3<> zmp_acc; // ZMP
  Vector3<> ZMP_WCS; // ZMP in Walking Engine World Coordinate System

  /** Constructor */
  ZMPModel() : zmp_acc(Vector3<>()), ZMP_WCS(Vector3<>()) {}
};
