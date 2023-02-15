/**
 * @file ScoreGoal2v2.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(ScoreGoal2v2) {

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
                          gloBall.y >= theFieldDimensions.yPosLeftPenaltyArea + theFieldDimensions.goalPostRadius;

  const float goalDist = (goalCenter - gloBall).abs();
  const Range<float> goalRange = {(goalRight - gloBall).angle(), (goalLeft - gloBall).angle()};

  std::list<Range<float>> freeRanges;
  Vector2<> kickTarget;

  // Parameters all in global frame
  const auto calculateFreeRanges =
    [](const Vector2<>& ball, const Range<float> goalRange, const std::vector<Vector2<>>& players) {
      // Represents a union of intervals of angles (defined in global frame centered on the ball)
      // that represent free regions when aiming at the goal
      std::list<Range<float>> freeRanges = {goalRange};

      // Subtract a free interval for every player
      for (const auto& player : players) {
        const Vector2<> ballToPlayer = player - ball;
        const float playerRadius = 240.f; // mm; prefer smaller numbers (to avoid making robots block too much range)
        const float blockedAngle = std::asin(playerRadius / ballToPlayer.abs());

        const Range<float> blockedRange = {ballToPlayer.angle() - blockedAngle, ballToPlayer.angle() + blockedAngle};

        // Ignore players completely outside of our problem
        if (blockedRange < goalRange || blockedRange > goalRange) {
          continue;
        }

        // Remove blockedRange from [freeRanges]
        for (auto i = freeRanges.begin(); i != freeRanges.end();) {
          auto& range = *i;
          if (blockedRange.contains(range)) {
            // Delete original range
            i = freeRanges.erase(i);

          } else if (range.contains(blockedRange)) {
            // Split original range into two
            const Range<float> range1(range.min, blockedRange.min);
            const Range<float> range2(blockedRange.max, range.max);

            i = freeRanges.erase(i);
            i = freeRanges.insert(i, range1);
            ++i;
            i = freeRanges.insert(i, range2);
            ++i;

          } else {
            if (range.overlaps(blockedRange)) {
              // Cut back "max"-side of original range
              range.max = blockedRange.min;
            } else if (range.overlappedBy(blockedRange)) {
              // Cut back "min"-side of original range
              range.min = blockedRange.max;
            }
            ++i;
          }
        }
      }

      return freeRanges;
    };

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
      if (goalDist < tacticalThreshold) {
        goto tactical;
      }
    }

    action {
      kickTarget = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal};
      KickTo2v2(kickTarget, preciseTol);
    }
  }

  // Tactical shot: aim for undefended / free areas
  state(tactical) {
    transition {
      if (goalDist >= tacticalThreshold) {
        goto longShot;
      }
    }

    action {
      std::vector<Vector2<>> gloPlayers;
      for (const auto& player : thePlayerModel.players) {
        gloPlayers.push_back(theRobotPose * player.relPosOnField);
      }
      freeRanges = calculateFreeRanges(gloBall, goalRange, gloPlayers);

      // Aim towards widest free range; break close ties by angular proximity
      float kickDirection;
      if (freeRanges.empty()) {
        kickDirection = goalRange.getCenter();
      } else {
        const auto bestFreeRange = *std::max_element(
          freeRanges.begin(), freeRanges.end(), [this, &gloBall](const Range<float>& a, const Range<float>& b) {
            // Reward wide ranges, lightly penalize those far away
            const float robotToBallAngle = (gloBall - theRobotPose.translation).angle();
            const float rewardA = a.getSize() - 0.15f * std::abs(angleDifference(robotToBallAngle, a.getCenter()));
            const float rewardB = b.getSize() - 0.15f * std::abs(angleDifference(robotToBallAngle, b.getCenter()));
            return rewardA < rewardB;
          });

        // Aim towards center of best free range, or as close to a goalpost as possible (possible SIM ARTIFACT)
        kickDirection = bestFreeRange.getCenter();
      }

      kickTarget = gloBall + Vector2<>(4000.f, 0.f).rotated(kickDirection);
      KickTo2v2(kickTarget, preciseTol);
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
      KickTo2v2(kickTarget, preciseTol);
    }
  }
}
