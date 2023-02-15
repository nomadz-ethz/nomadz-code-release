/**
 * @file DribblingParameters.h
 *
 * Declaration of all parameters of dribbling
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

/**
 * A collection of all parameters for dribbling.
 */
STREAMABLE(
  DribblingParameters,
  {
    ,
    (float)(-25)dribblingMinBallOffset,       /* Minimal offset of robot relative to the ball while dribbling */
    (float)(32.5)dribblingMaxBallOffset,      /* Maximal offset of robot relative to the ball while dribbling */
    (float)(160)dribblingMinBallToTargetDist, /* Minimal distance between ball and target (for calculation) */
    (float)(640)dribblingMaxBallToTargetDist, /* Maximal distance between ball and target (for calculation) */
    (float)(200)dribblingViapointBallDist,    /* Distance from ball to via-point, the robot walks when it is between the ball
                                                 and the target */
    (float)(195)
      dribblingThresholdHeading, /* Distance from the robot to the ball before the robot is heading aligning point */
    (float)(200)
      dribblingThreshDistBeforeDribble, /* Distance from the robot to the ball before the robot begins to dribble it */
    (float)(100)
      dribblingTargetAccuracy, /* Tolerance to decide if the ball has reached the desired dribbling target or not */
  });
