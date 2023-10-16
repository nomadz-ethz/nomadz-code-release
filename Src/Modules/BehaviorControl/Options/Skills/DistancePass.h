/**
 * @file DistancePass.h
 *
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 * Description: This skill implements a pass over a given distance [mm]
 * Note: The robot should be aligned to it's target before this skill is called
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Core/Debugging/DebugDrawings3D.h"

// Define Global Variables
bool takeLeftFoot;

float u_r = 0.05f;
float c1 = -0.0427;
float c2 = 1.0762;
float c3 = -0.0845;

option(DistancePass, float distanceToTarget) {
  float kickStrength = 0.f;
  float kickVelocity = 0.f;
  float distance = 0.f; // [m]

  // convert the distance to [m]
  distance = distanceToTarget / 1000.0; // The distance of the target relative to the robot (in m)

  // Calculate the kick-strength which should be applied to kick the ball over the desired distance
  kickVelocity = sqrt(2 * u_r * 9.81 * distance);

  // return the kick-strength (c1, c2 and c3 are line parameters because the kick-strength is not linearly dependent on the
  // ball velocity)
  kickStrength = c1 + c2 * kickVelocity + c3 * sqr(kickVelocity);
  std::cout << "kickStrength: " << kickStrength << std::endl;

  // Always executed
  common_transition {}

  initial_state(calculateKickParameters) {
    transition {
      // wait until the kick strength is calculated
      if (kickStrength != 0.f) {
        goto kickTheBall;
      }
    }
    action { StandHigh(); }
  }

  state(kickTheBall) {
    transition {
      if (state_time > 50 && theBallModel.estimate.velocity.abs() > 0.0) {
        goto successfullKicked;
      }

      if (state_time > 5000) {
        goto failedToKick;
      }
    }

    action {
      // Choose which foot should be used to pass the ball
      if (theBallModel.estimate.position.y > 0) { // decide whether it should take left or right foot
        takeLeftFoot = true;                      // take left foot
      } else {
        takeLeftFoot = false; // take right foot
      }

      theMotionRequest.kickRequest.kickMotionType = KickRequest::kickForward;
      theMotionRequest.kickRequest.mirror = takeLeftFoot;
      // FIXME: no correct kick strength
      theMotionRequest.motion = MotionRequest::kick;
    }
  }

  /** kick is done. */
  target_state(successfullKicked) {}

  // kick aborted: because ball was not kicked to the robot *
  aborted_state(failedToKick) {}
}