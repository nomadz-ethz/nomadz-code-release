/**
 * @file GetBehindBall.h
 *
 * Walk to somewhere behind the ball, ... and keep walking towards that point behind the ball.
 * On the way to the ball, walks around other players to avoid collision.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(GetBehindBall,
       Vector2<> target,
       Vector2<> offset,
       Pose2D tolSmall = {fromDegrees(7.5f), 40.f, 25.f},
       Pose2D tolBig = {fromDegrees(7.5f), 50.f, 35.f}) {

  // Sanitize input tolerances by seting out of bound values to the default tolerances
  if (tolSmall.rotation <= 0.f) {
    tolSmall.rotation = fromDegrees(7.5f);
  }
  if (tolSmall.translation.x <= 0.f) {
    tolSmall.translation.x = 40.f;
  }
  if (tolSmall.translation.y <= 0.f) {
    tolSmall.translation.y = 25.f;
  }

  if (tolBig.rotation <= 0.f) {
    tolBig.rotation = fromDegrees(7.5f);
  }
  if (tolBig.translation.x <= 0.f) {
    tolBig.translation.x = 50.f;
  }
  if (tolBig.translation.y <= 0.f) {
    tolBig.translation.y = 35.f;
  }
  // callculate the target position of the robot and adjust them with ofsets for the left/right kick
  const Vector2<> gloBall = relToGlo(theBallModel.estimate.position);
  const Pose2D targetFrame((target - gloBall).angle(), gloBall); // global

  const Pose2D kickPoseLeftFoot = targetFrame + Pose2D(0.f, -offset.x, -offset.y); // global (offset rotated into kick frame)
  const Pose2D kickPoseRightFoot = targetFrame + Pose2D(0.f, -offset.x, +offset.y);
  Pose2D kickPose = kickPoseRightFoot;

  // in between pose to walk around the ball
  Pose2D waypoint;

  // decide based on the robot position and the goal position which foot to align for a shot or which way to walk around the
  // ball
  float norminalTarget = (targetDistanceRange.min + targetDistanceRange.max) / 2;
  const float ballAngle = (gloBall - theRobotPose.translation).angle();
  const Pose2D targetFrame2(ballAngle,
                            gloBall - Vector2<>(norminalTarget * cos(ballAngle), norminalTarget * sin(ballAngle))); // global

  initial_state(decide) {
    transition {
      if (theBallModel.estimate.position.abs() < norminalTarget + 20) {
        goto circleAround;
      }
    }
    action { TravelTo(targetFrame2); }
  }

  state(circleAround) {
    transition {
      if (action_done) {
        if ((theRobotPose - targetFrame).translation.y >= 0.f) {
          // Behind & left of ball
          goto alignRightFoot;

        } else {
          // Behind & right of ball
          goto alignLeftFoot;
        }
      }
    }
    action {
      Angle globalTargetAngle = (target - gloBall).angle();
      PatternCircleAround(globalTargetAngle);
    }
  }

  // try to align the left foot for a kick
  state(alignLeftFoot) {
    kickPose = kickPoseLeftFoot;

    transition {
      const Pose2D deviation = theRobotPose - kickPose;

      if (isWithinTol(deviation, tolSmall) || (state_time > 3000 && isWithinTol(deviation, tolBig))) {
        goto stationaryAligmentCheck;

      } else if ((theRobotPose - targetFrame).translation.x > 100.f) {
        // Gotta go around the back again
        goto decide;

      } else if (deviation.translation.y > +std::abs(offset.y) + 50.f) {
        // Too far to the left
        goto alignRightFoot;
      }
    }

    action { TravelTo(kickPose); }
  }

  // align right foot to kick
  state(alignRightFoot) {
    kickPose = kickPoseRightFoot;

    transition {
      const Pose2D deviation = theRobotPose - kickPose;

      if (isWithinTol(deviation, tolSmall) || (state_time > 3000 && isWithinTol(deviation, tolBig))) {
        goto stationaryAligmentCheck;

      } else if ((theRobotPose - targetFrame).translation.x > 100.f) {
        // Gotta go around the back again
        goto decide;

      } else if (deviation.translation.y < -std::abs(offset.y) - 50.f) {
        // Too far to the right
        goto alignLeftFoot;
      }
    }

    action { TravelTo(kickPose); }
  }

  state(stationaryAligmentCheck) {
    transition {
      if (state_time > 200) {
        if (theBallModel.estimate.position.y >= 0) {
          if (!isWithinTol(theRobotPose - kickPoseLeftFoot, tolBig)) {
            goto alignLeftFoot;
          }
        } else {
          if (!isWithinTol(theRobotPose - kickPoseRightFoot, tolBig)) {
            goto alignRightFoot;
          }
        }
        goto success;
      }
    }

    action { Stand(); }
  }

  target_state(success) {}
  SPHERE3D_VEC("module:validRegion", Vector3<>(gloBall[0], gloBall[1], 0), 20, ColorClasses::black);
};
