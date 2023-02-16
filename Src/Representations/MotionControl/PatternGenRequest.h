/**
 * @file PatternGenRequest.h
 *
 * Contains the request for the pattern genertor, e.g. the desired speed and the next walk state.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once
#ifndef WALKING_SIMULATOR
#include "Core/Math/Pose2D.h"
#include "Core/Streams/AutoStreamable.h"
#include "Core/Debugging/DebugDrawings.h"
#else
#include "math/Pose2D.h"
#include "bhumanstub.h"
#endif
#include "Core/Enum.h"
/**
 * @class PatternGenRequest
 * Contains the request for the pattern genertor, e.g. the desired speed and the next walk state.
 */
STREAMABLE_DECLARE_LEGACY(PatternGenRequest)
class PatternGenRequest : public PatternGenRequestBaseWrapper {
public:
  /** Constructor */
  PatternGenRequest() {
    newState = NA;
    speed.translation.x = 0;
    speed.translation.y = 0;
    speed.rotation = 0;
    pitch = 0;
  };

  /** Desired pitch */
  float pitch;

  /** Desired speed */
  Pose2D speed;

  /** Desired walking state*/
  ENUM(State,
       standby,       /**< Walking engine (ZMP/IP-Controller) not active. Hard coded foot positions used.*/
       ready,         /**< Set the desired height of center of mass, run ZMP/IP-Controller, but stand still. */
       walking,       /**< Walk by using the ZMP/IP-Controller */
       NA,            /**< Unknown request */
       emergencyStop, /**< Stop NOW, e.g. in case of falling */
       standLeft,
       standRight);

  /** The requested new state */
  State newState;

  bool deceleratedMax;       /** Maximum speed exceeded. */
  bool deceleratedStability; /** Decelerated due to unstability. */

#ifndef WALKING_SIMULATOR

  /**
   * The method draws the path.
   */
  void draw() const {
    DECLARE_DEBUG_DRAWING("representation:PatternGenRequest", "drawingOnField");
    // draw next path points (relative to RobotPoseAfterPreview!)
    Pose2D last, next;
    Pose2D speed_(speed);
    float factor = 1000; // speed is in m/s, but drawing in mm
    speed_.translation *= factor;
    const int n = 12;
    const int arrowEveryN = 3;
    for (int i = 0; i < n; i++) {
      last = next;
      next += speed_;
      if ((i % arrowEveryN) == arrowEveryN - 1) {
        ARROW("representation:PatternGenRequest",
              last.translation.x,
              last.translation.y,
              next.translation.x,
              next.translation.y,
              10,
              Drawings::ps_solid,
              ColorRGBA(255, 0, 255));
      } else {
        LINE("representation:PatternGenRequest",
             last.translation.x,
             last.translation.y,
             next.translation.x,
             next.translation.y,
             10,
             Drawings::ps_solid,
             ColorRGBA(255, 0, 255));
      }
    }
  }
#endif

protected:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(speed)
    STREAM(newState);
    STREAM_REGISTER_FINISH;
  }
};
