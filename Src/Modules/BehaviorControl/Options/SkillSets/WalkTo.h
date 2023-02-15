/**
 * @file WalkTo.h
 *
 * This skill can be called any where in the behaviour to make the robot walk
 * to a a particular global target position and orientation. It tries to do it so that
 * we minimize the side walking.
 *
 * target: global rotation & position we want to be in
 * stopAtTarget: true if we want to stop at "target" as precisely as possible, false if we keep moving forward once we're
 * there
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <algorithm>
#include <math.h>

option(WalkTo, Pose2D target, bool stopAtTarget = true) {

  const Pose2D relTarget = target - theRobotPoseAfterPreview;

  // forward speed (200 mm/s) / turn speed (2 rad/s) = 100 mm / rad => 100.f
  const float alignThresholdX = std::max(200.f, std::abs(relTarget.rotation) * 100.f);
  const Rangef sideWalkRangeY(100.f, 500.f);
  const Rangef sideWalkRangeX(50.f, 1000.f);
  const float sonarObstacleThreshhold = 300;
  bool avoidToLeft = true;
  RingBufferWithSum<int, 3> usR;
  RingBufferWithSum<int, 3> usL;

  common_transition {
    usR.add(theSensorData.data[SensorData::usR]);
    usL.add(theSensorData.data[SensorData::usL]);
    if (theFootContactModel.contactRight || usR.getAverage() < sonarObstacleThreshhold) {
      avoidToLeft = false;
      PlaySound("buf.wav");
      goto sideStep;
    } else if (theFootContactModel.contactLeft || usL.getAverage() < sonarObstacleThreshhold) {
      avoidToLeft = true;
      PlaySound("buf.wav");
      goto sideStep;
    }
  }

  initial_state(rotate) {
    transition {
      if (std::abs(relTarget.translation.x) < alignThresholdX && std::abs(relTarget.translation.y) < 150.f) {
        goto align;
      } else if (std::abs(angleToGlobalTarget(target.translation)) < fromDegrees(45.f)) {
        goto walk;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(1.0f, 0.4f, 0.f),
                   Pose2D(angleToGlobalTarget(target.translation), theRobotPoseAfterPreview.invert() * target.translation));
    }
  }

  state(sideStep) {
    transition {
      if (state_time > 500) {
        goto walk;
      }
    }
    action {
      if (avoidToLeft) {
        theArmMotionRequest.motion[ArmMotionRequest::left] = ArmMotionRequest::back;
        if (theFootContactModel.contactLeft) {
          WalkToTarget(Pose2D(0.0f, 1.0f, 1.0f), Pose2D(0, -50, -200));
        } else {
          WalkToTarget(Pose2D(0.0f, 1.0f, 1.0f), Pose2D(0, 80, -200));
        }
      } else {
        theArmMotionRequest.motion[ArmMotionRequest::right] = ArmMotionRequest::back;
        if (theFootContactModel.contactRight) {
          WalkToTarget(Pose2D(0.0f, 1.0f, 1.0f), Pose2D(0, -50, 200));
        } else {
          WalkToTarget(Pose2D(0.0f, 1.0f, 1.0f), Pose2D(0, 80, 200));
        }
      }
    }
  }
  state(walk) {
    transition {
      if (std::abs(relTarget.translation.x) < alignThresholdX && std::abs(relTarget.translation.y) < 150.f) {
        goto align;
      } else if (std::abs(angleToGlobalTarget(target.translation)) > fromDegrees(20.f)) {
        goto rotate;
      }
    }
    action {
      if (state_time > 1000) {
        theArmMotionRequest.motion[ArmMotionRequest::left] = ArmMotionRequest::useDefault;
        theArmMotionRequest.motion[ArmMotionRequest::right] = ArmMotionRequest::useDefault;
      }
      WalkToTarget(Pose2D(0.1f, 1.0f, 0.4f),
                   Pose2D(angleToGlobalTarget(target.translation), theRobotPoseAfterPreview.invert() * target.translation));
    }
  }

  state(align) {
    transition {
      if (std::abs(relTarget.translation.x) > 350.f || std::abs(relTarget.translation.y) > 500.f) {
        goto walk;
      }
    }
    action {
      // If !stopAtTarget, extend relative target distance by ~1m
      float mult = 0;
      if (relTarget.translation.abs() > 1e-3) {
        mult = 1000.f / relTarget.translation.abs();
      }
      const Pose2D virtualTarget =
        stopAtTarget ? relTarget : Pose2D(relTarget.rotation, relTarget.translation * (1.f + mult));

      // Slow down forward/backward speed when near
      const float dx = virtualTarget.translation.x;
      float maxSpeedX;
      maxSpeedX = std::min(0.7f * abs(dx) / 150 + 0.3f, 1.0f);

      // Don't limit x-speed if not stopping at target
      if (!stopAtTarget) {
        maxSpeedX = 1.f;
      }

      // Slow down sideways speed when sideways distance gets close
      const float dy = virtualTarget.translation.y;
      float maxSpeedY;
      maxSpeedY = std::min(0.7f * abs(dy) / 150 + 0.3f, 1.f);

      // Slow down rotation speed when target angle gets close
      const float dr = std::abs(virtualTarget.rotation);
      float maxSpeedR;
      if (dr < fromDegrees(10.f)) {
        maxSpeedR = 0.3f;
      } else {
        maxSpeedR = 1.f;
      }

      WalkToTarget(Pose2D(maxSpeedR, maxSpeedX, maxSpeedY), virtualTarget);

      COMPLEX_DRAWING("behavior:Field", {
        const ColorRGBA pathColor(0xff, 0, 0);

        // Draw virtualTarget & dashed line to it
        const Pose2D gloTarget = theRobotPoseAfterPreview + virtualTarget;
        const std::vector<Vector2<>> points = {theRobotPoseAfterPreview.translation,
                                               gloTarget.translation,
                                               gloTarget.translation + Vector2<>(300.f, 0.f).rotate(gloTarget.rotation)};
        LINE("behavior:Field", points[0].x, points[0].y, points[1].x, points[1].y, 5, Drawings::ps_dash, pathColor);
        CROSS("behavior:Field", points[1].x, points[1].y, 40, 5, Drawings::ps_solid, pathColor);
        ARROW("behavior:Field", points[1].x, points[1].y, points[2].x, points[2].y, 5, Drawings::ps_solid, pathColor);
      });
    }
  }
}

bool ballInFront(float tol = 0.f) {
  return Rangef(-100.f + tol, 100.f - tol).contains(theBallModelAfterPreview.estimate.position.y) &&
         Rangef(-150.f + tol, 750.f - tol).contains(theBallModelAfterPreview.estimate.position.x);
}
