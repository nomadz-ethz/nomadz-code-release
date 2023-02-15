/**
 * @file OmniGetBehindBall.h
 *
 * Walk to somewhere behind the ball, ... and keep walking towards that point behind the ball.
 * On the way to the ball, walks around other players to avoid collision.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(OmniGetBehindBall,
       Vector2<> target,
       Vector2<> offset,
       bool usingDefault = true,
       Pose2D tolSmall = {fromDegrees(7.5f), 50.f, 35.f},
       Pose2D tolBig = {fromDegrees(15.f), 75.f, 50.f}) {
  // TODO: tune the tolerances. To reduce the aligning time, it should be set a bit higher.
  // At the same time the offset should be set a bit lower so that the ball will still be in
  // a relatively ideal position for the kick

  // Sanitize input tolerances by seting out of bound values to the default tolerances
  if (tolSmall.rotation <= 0.f) {
    tolSmall.rotation = fromDegrees(7.5f);
  }
  if (tolSmall.translation.x <= 0.f) {
    tolSmall.translation.x = 50.f;
  }
  if (tolSmall.translation.y <= 0.f) {
    tolSmall.translation.y = 35.f;
  }

  if (tolBig.rotation <= 0.f) {
    tolBig.rotation = fromDegrees(15.f);
  }
  if (tolBig.translation.x <= 0.f) {
    tolBig.translation.x = 75.f;
  }
  if (tolBig.translation.y <= 0.f) {
    tolBig.translation.y = 50.f;
  }

  const float durableOmniAngle = fromDegrees(45.f);

  // callculate the target position of the robot and adjust them with offsets for the left/right kick
  Vector2<> gloBall = relToGlo(theBallModel.estimate.position);
  float targetAngle = (target - gloBall).angle(); // angle Ball -> Target: (-pi, pi)

  Vector2<> robotToBall = gloBall - theRobotPose.translation;
  float kickPoseRot = robotToBall.angle();               // angle Robot ->  Ball
  float diffAngle = angleDiff(kickPoseRot, targetAngle); // kickPoseRot - targetAngle

  if (abs(diffAngle) >= durableOmniAngle) {
    float angle1 = angleSum(targetAngle, -durableOmniAngle);
    float angle2 = angleSum(targetAngle, durableOmniAngle);
    kickPoseRot = (diffAngle < 0) ? angle1 : angle2;
  }

  float offsetY;
  if (usingDefault) {
    offsetY = offset.y * (1.f - powf(3 * abs(diffAngle) / pi, 2));
  } else {
    offsetY = offset.y * (1.f - powf(1.5 * abs(diffAngle) / pi, 2));
  }
  float offsetRot = asin(Rangef(-1.f, 1.f).limit(offsetY / robotToBall.abs()));
  kickPoseRot += (diffAngle > 0.f) ? -offsetRot : offsetRot;
  // OUTPUT_TEXT("rot = " << kickPoseRot << ", diff > durable = " << (abs(diff_angle) >= durableOmniAngle));
  // OUTPUT_TEXT("targetAngle = " << target_angle * 180 / pi);

  Vector2<> kickPoseTrans = gloBall - Vector2<>(offset.x, 0.f).rotate(kickPoseRot);
  Vector2<> offsetTransY = Vector2<>(0, offsetY).rotate(kickPoseRot);
  kickPoseTrans += (diffAngle < 0.f) ? offsetTransY : -offsetTransY;

  Pose2D kickPose(kickPoseRot, kickPoseTrans); // global

  /**
   * @remark if the durableOmniAngle is big enough, e.g. 80 deg (should be smaller than 90 deg and considering the tolerance,
   * better to set it at around 80 deg as maximum), the robot can directly walk to the kick position and two states are
   * enough for this behavior. Nevertheless, a lower durable angle needs an extra state "walkToWaypoint" so that the robot
   * will not touch the ball when walking to the kick position.
   *
   * As a result, comment the "initial_state" and "target_state" below and uncomment the rest of the code when
   * durableOmniAngle is small and the other way around when durableOmniAngle is set high
   *
   */
  // initial_state(walkToTargetFrame) {
  //   transition {
  //     const Pose2D deviation = theRobotPose - kickPose;

  //     if (isWithinTol(deviation, tolSmall) || (state_time > 3000 && isWithinTol(deviation, tolBig))) {
  //       goto reachedTargetFrame;
  //     }
  //   }

  //   action {
  //     theHeadControlMode = HeadControl::lookAtBall;
  //     TravelTo(kickPose);
  //   }
  // }

  // target_state(reachedTargetFrame) {
  //   transition {
  //     const Pose2D deviation = theRobotPose - kickPose;

  //     if (!isWithinTol(deviation, tolBig)) {
  //       goto walkToTargetFrame;
  //     }
  //   }

  //   action {
  //     theHeadControlMode = HeadControl::lookAtBall;
  //     TravelTo(kickPose);
  //   }
  // }

  /**
   * @remark: if durableOmniAngle is set kinda small, e.g. 45 deg, then the state machine has to contain one extra state
   * "walkToWaypoint" so that during the way the robot walks to the kick position, it won't touch the ball. But when the
   * durable angle is set to be very high (should be lower than 90 deg), it doen't matter anymore.
   *
   * So if the kick engine is not stable with high kick angle, uncomment the code below
   * and comment the "initial_state" and "target_state" above
   *
   */
  // distance betwen robot and ball to make sure to not accidentally hit it
  const float avoidanceDistance = 240.f;
  Vector2<> waypointPos;
  if (diffAngle > 0) {
    waypointPos.x = gloBall.x + avoidanceDistance * cosf(angleSum(targetAngle, -pi_2));
    waypointPos.y = gloBall.y + avoidanceDistance * sinf(angleSum(targetAngle, -pi_2));
  } else {
    waypointPos.x = gloBall.x + avoidanceDistance * cosf(angleSum(targetAngle, pi_2));
    waypointPos.y = gloBall.y + avoidanceDistance * sinf(angleSum(targetAngle, pi_2));
  }

  float temp = (kickPoseTrans - waypointPos).abs();
  float extraAngle = acos((sqr(avoidanceDistance) + sqr(temp) - sqr(offset.x)) / (2 * avoidanceDistance * temp));
  float waypointAngle =
    (diffAngle > 0) ? angleSum(targetAngle, pi_2 + extraAngle) : angleSum(targetAngle, -pi_2 - extraAngle);

  Pose2D waypoint(waypointAngle, waypointPos);

  // decide based on the robot position and the goal position which foot to align for a shot or which way to walk around the
  // ball
  initial_state(decide) {
    transition {
      if (abs(diffAngle) <= pi_2) {
        goto walkToTargetFrame;
      } else {
        goto walkToWaypoint;
      }
    }
  }

  state(walkToWaypoint) {
    transition {
      const Pose2D deviation = theRobotPose - waypoint;
      deviation.rotation = 0.f;

      if (isWithinTol(deviation, tolSmall) || (state_time > 1000 && isWithinTol(deviation, tolBig)) ||
          abs(diffAngle) <= pi_2) {
        goto walkToTargetFrame;
      }
    }

    action {
      theHeadControlMode = HeadControl::lookAtBall;
      TravelTo(waypoint);
    }
  }

  state(walkToTargetFrame) {
    transition {
      const Pose2D deviation = theRobotPose - kickPose;

      if (isWithinTol(deviation, tolSmall) || (state_time > 3000 && isWithinTol(deviation, tolBig))) {
        goto reachedTargetFrame;
      } else if (abs(diffAngle) > pi_2 + fromDegrees(20.f)) {
        // with a 10[deg] toletance to avoid oscilation between 'walkToTargetFrame' & 'walkToWaypoint'
        goto walkToWaypoint;
      }
    }

    action {
      theHeadControlMode = HeadControl::lookAtBall;
      TravelTo(kickPose);
    }
  }

  target_state(reachedTargetFrame) {
    transition {
      const Pose2D deviation = theRobotPose - kickPose;

      if (!isWithinTol(deviation, tolBig)) {
        goto walkToTargetFrame;
      }
    }

    action {
      theHeadControlMode = HeadControl::lookAtBall;
      TravelTo(kickPose);
    }
  }
}

float angleDiff(float alpha, float beta) {
  float diff = alpha - beta;
  if (beta < pi && diff > pi) {
    diff -= pi2;
  } else if (beta > pi && diff < -pi) {
    diff += pi2;
  }
  return diff;
}

float angleSum(float alpha, float beta) {
  return wrapAngle(alpha + beta);
}
