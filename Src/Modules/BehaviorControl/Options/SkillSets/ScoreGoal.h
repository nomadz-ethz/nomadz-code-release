/**
 * @file ScoreGoal.h
 *
 * Walk to the ball and try to score a goal, aiming towards the closest large free space in the opponent goal if possible.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(ScoreGoal) {

  // Estimate of size of cone where the kick could go (+/- "accuracy" radians); be conservative
  const float preciseAccuracy = fromDegrees(15.f);

  // Tolerances for positioning in KickTo
  const Pose2D preciseTol = {fromDegrees(7.f), 30.f, 20.f};

  // Distance thresholds
  const float tacticalThreshold =
    (theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius) / std::tan(preciseAccuracy);

  const Vector2<> gloBall = theRobotPose * theBallModel.estimate.position;
  const Vector2<> goalCenter = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal};
  const Vector2<> goalLeft = {theFieldDimensions.xPosOpponentGroundline,
                              theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius};
  const Vector2<> goalRight = {theFieldDimensions.xPosOpponentGroundline,
                               theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius};

  const bool inDeadArea = gloBall.x >= theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.goalPostRadius &&
                          gloBall.y >= theFieldDimensions.yPosLeftGoalBox + theFieldDimensions.goalPostRadius;

  const float goalDist = (goalCenter - gloBall).abs();
  const Range<float> goalRange = {(goalRight - gloBall).angle(), (goalLeft - gloBall).angle()};

  std::list<Range<float>> freeRanges;
  Vector2<> kickTarget;

  common_transition {
    if (inDeadArea) {
      // Important: we must not be kicking towards +/- 180 deg (towards own goal), since the free ranges
      // logic can't handle the discontinuity at +/- 180 deg.
      // Also, if we're beside the goal near the ground line, a different strategy is needed.
      goto deadArea;
    }
  }

  // Far away: aim for center of goal
  initial_state(longShot) {
    transition {
      if (shouldRush()) {
        goto rush;
      } else if (goalDist < tacticalThreshold) {
        goto tactical;
      }
    }

    action {
      freeRanges = libCodeRelease.calculateFreeRanges(gloBall, goalRange);

      // Aim towards widest free range; break close ties by angular proximity
      float kickDirection;
      if (freeRanges.empty()) {
        kickDirection = goalRange.getCenter();
      } else {
        const float robotToBallAngle = (gloBall - theRobotPose.translation).angle();
        const auto bestFreeRange = libCodeRelease.getBestFreeRange(freeRanges, robotToBallAngle);
        // Aim towards center of best free range, or as close to a goalpost as possible (possible SIM ARTIFACT)
        kickDirection = bestFreeRange.getCenter();
      }

      kickTarget = gloBall + Vector2<>(4000.f, 0.f).rotated(kickDirection);
      kickTarget = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal};
      KickTo(kickTarget, preciseTol);
    }
  }

  // Tactical shot: aim for undefended / free areas
  state(tactical) {
    transition {
      if (shouldRush()) {
        goto rush;
      } else if (goalDist >= tacticalThreshold) {
        goto longShot;
      }
    }

    action {
      freeRanges = libCodeRelease.calculateFreeRanges(gloBall, goalRange);

      // Aim towards widest free range; break close ties by angular proximity
      float kickDirection;
      if (freeRanges.empty()) {
        kickDirection = goalRange.getCenter();
      } else {
        const float robotToBallAngle = (gloBall - theRobotPose.translation).angle();
        const auto bestFreeRange = libCodeRelease.getBestFreeRange(freeRanges, robotToBallAngle);

        // Aim towards center of best free range, or as close to a goalpost as possible (possible SIM ARTIFACT)
        kickDirection = bestFreeRange.getCenter();
      }

      kickTarget = gloBall + Vector2<>(4000.f, 0.f).rotated(kickDirection);
      // KickTo(kickTarget, preciseTol);
      KickTo(kickTarget, preciseTol);
    }
  }

  // Quickly try to get the ball somewhere in the goal (considering rushAccuracy)
  // Only if really close to goal
  state(rush) {
    transition {
      if (goalDist >= tacticalThreshold) {
        goto longShot;
      } else if (!shouldRush()) {
        goto tactical;
      }
    }

    action {
      // Project ball onto goal line (+1000.f to account for ball position noise :) ), then get relative angles to goal posts
      const Vector2<> projected(theFieldDimensions.xPosOpponentGroundline + 1000.f, gloBall.y);
      const float leftTol = goalRange.max;
      const float rightTol = -goalRange.min;
      DribbleTo(projected, leftTol, rightTol);
    }
  }

  // Fall back to default behavior (while communicating to higher state that ScoreGoal will fail to score a goal)
  aborted_state(deadArea) {
    transition {
      if (!inDeadArea) {
        goto longShot;
      }
    }
    action {
      kickTarget = goalCenter;
      KickTo(kickTarget, preciseTol);
    }
  }

  COMPLEX_DRAWING("behavior:Field", {
    const float arcRadius = 600.f;
    for (const auto& range : freeRanges) {
      ASSERT(range.min <= range.max);

      // Draw 3 tick marks
      const std::vector<float> tickAngles = {range.min, range.getCenter(), range.max};
      for (float r : tickAngles) {
        const float tickLength = (r == range.getCenter()) ? 15.f : 50.f;
        std::vector<Vector2<>> pts = {{-0.5f, 0.f}, {0.5f, 0.f}};
        for (auto& pt : pts) {
          pt = gloBall + (pt * tickLength + Vector2<>(arcRadius, 0.f)).rotated(r);
        }

        LINE("behavior:Field", pts[0].x, pts[0].y, pts[1].x, pts[1].y, 8, Drawings::ps_solid, ColorRGBA(196, 0, 0));
      }

      // Draw part of the arc
      const float dr = fromDegrees(3.f);
      float r = range.min;
      while (r < range.max) {
        const Vector2<> pt1 = gloBall + Vector2<>(arcRadius, 0.f).rotated(r);
        const Vector2<> pt2 = gloBall + Vector2<>(arcRadius, 0.f).rotated(std::min(r + dr, range.max));
        LINE("behavior:Field", pt1.x, pt1.y, pt2.x, pt2.y, 8, Drawings::ps_solid, ColorRGBA(196, 0, 0));
        r += dr;
      }
    }

    const Vector2<> kickVector = (kickTarget - gloBall).direction();
    ARROW("behavior:Field",
          gloBall.x,
          gloBall.y,
          (gloBall + kickVector * arcRadius).x,
          (gloBall + kickVector * arcRadius).y,
          5,
          Drawings::ps_dash,
          ColorRGBA(196, 0, 0));
  });
}

bool shouldRush() {
  const float rushDistance = 750.f;
  const Vector2<> gloBall = theRobotPose * theBallModel.estimate.position;
  return gloBall.x > theFieldDimensions.xPosOpponentGroundline - rushDistance &&
         std::abs(gloBall.y) < theFieldDimensions.yPosLeftGoal;
}
