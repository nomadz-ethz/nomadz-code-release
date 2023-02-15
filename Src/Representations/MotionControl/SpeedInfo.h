/**
 * @file SpeedInfo.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#pragma once

#include "Tools/DortmundWalkingEngine/Point.h"
#include "Representations/MotionControl/WalkRequest.h"
#ifndef WALKING_SIMULATOR
#include "Core/Streams/AutoStreamable.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class SpeedInfo
 * Contains information about the actual speed.
 */
STREAMABLE_DECLARE_LEGACY(SpeedInfo)
struct SpeedInfo : public SpeedInfoBaseWrapper {
  Pose2D speed;              /**< The< actual speed. */
  Pose2D speedBeforePreview; /**< Future speed in ControllerParams::N frames */
  bool deceleratedByAcc{};   /** Decelerated due to limited acceleration */
  int timestamp{};           /** Timestamp of the foot executing this speed */

  /** Currently executed custom step file. If this is not "none" the speed above
   is invalid. */
  WalkRequest::StepRequest currentCustomStep;

  int delay{}; /**< The current speed was requested delay frames before. */

  /** Constructor */
  SpeedInfo() {}

  /** Destructor */
  ~SpeedInfo() {}

protected:
  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM_REGISTER_FINISH;
  }
};
