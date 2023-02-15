/**
 * @file ControllerParams.h
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
#include "Core/Math/Vector3.h"
#include "Core/Math/Vector2.h"
#include "Core/Math/Matrix.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class ControllerParams
 *
 * Contains the parameters for the ZMP/IP-Controller calculated by the
 * matlab script writeParams.m
 */
STREAMABLE_DECLARE_LEGACY(ControllerParams)
class ControllerParams : public ControllerParamsBaseWrapper {

public:
  float dt;                /**< Duration of one frame */
  float z_h;               /**< desired CoM position (height) */
  static const int N = 50; /**< Length of the preview phase */
  Matrix<N, 1, float> Gd;  /**< Data for preview controller */
  Matrix3x3f A0;           /**< Data for preview controller */
  float Gi;                /**< Data for preview controller */
  Matrix<1, 3, float> Gx;  /**< Data for preview controller */
  Vector3f b0;             /**< Data for preview controller */
  Matrix<1, 3, float> c0;  /**< Data for preview controller */
  Matrix<3, 2, float> L;   /**< Data for preview controller */
  Vector3f Ge[N];          /**< Data for preview controller */

  /** Constructor */
  ControllerParams(){};

  /** Destructor */
  ~ControllerParams(){};

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(dt);
    STREAM(z_h);
    STREAM(Gd);
    STREAM(A0);
    STREAM(Gi);
    STREAM(Gx);
    STREAM(b0);
    STREAM(c0);
    STREAM(L);
    STREAM(Ge);
    STREAM_REGISTER_FINISH;
  };
};
