/**
 * @file CoordinatedPassParameters.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

// Parameter needed by the CoordinatedPassProvider and CoordinatedPass skills
STREAMABLE(CoordinatedPassParameters,
           {
             ,
             // set true if successfully passed the ball
             (bool)(false)ballKicked,

             // distance at which the robot should start the kick motion
             (double)(0)startKickMotion,

             // old distance from ball to me
             (float)(0.0)myDistanceToBallOld,

             // save the old ballVelocity
             (double)(0.0)ballVelocityOld,

             // Tell if new random target point should be found
             (bool)(false)getNewTargetPoint,
           });
