/**
 * @file EngageBall.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

Vector2<> gloKickEstimate;
bool ballKickedAway;

option(EngageBall) {
  const Vector2<> gloOpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);

  Vector2<> gloBallTarget =
    theRobotPose * theBallModel.estimate.position +
    Vector2<>(1000.f, 0.f).rotate(Angle(theRobotPose.rotation + theLocalSkill.nextTargetBallAngle).normalize());
  Angle gloBallTargetAngle = theRobotPose.rotation + theLocalSkill.nextTargetBallAngle;

  initial_state(decideNextAction) {
    transition {
      if (veryCloseToOpponentGoal() || veryCloseToOwnGoal()) {
        goto rush;
      }
      if (ballTooCloseToSide()) {
        goto corner;
      }
      switch (theLocalSkill.skillType) {
      case LocalSkill::inWalkKick:
        goto inWalk;
      case LocalSkill::fastKick:
        goto longKick;
      case LocalSkill::dribble:
        goto dribble;
      default:
        break;
      }
    }
    action { Stand(); }
  }

  state(inWalk) {
    transition {
      if (theBallModel.lost) {
        goto searchForBall;
      }
      if (veryCloseToOpponentGoal() || veryCloseToOwnGoal()) {
        goto rush;
      }
      if (ballTooCloseToSide()) {
        goto corner;
      }
      if (theLocalSkill.skillType != LocalSkill::inWalkKick) {
        goto decideNextAction;
      }
    }
    action { InWalkKickTo(gloBallTargetAngle); }
  }

  state(longKick) {
    transition {
      if (theBallModel.lost) {
        goto searchForBall;
      }
      if (veryCloseToOpponentGoal() || veryCloseToOwnGoal()) {
        goto rush;
      }
      if (ballTooCloseToSide()) {
        goto corner;
      }
      if (theLocalSkill.skillType != LocalSkill::fastKick) {
        goto decideNextAction;
      }
    }
    action { KickTo(gloBallTarget); }
  }

  state(dribble) {
    transition {
      if (theBallModel.lost) {
        goto searchForBall;
      }
      if (veryCloseToOpponentGoal() || veryCloseToOwnGoal()) {
        goto rush;
      }
      if (ballTooCloseToSide()) {
        goto corner;
      }
      if (theLocalSkill.skillType != LocalSkill::dribble) {
        goto decideNextAction;
      }
    }
    action {
      // gloBallTargetAngle
      InWalkKickTo(gloBallTargetAngle);
    }
  }

  state(rush) {
    transition {
      if (theBallModel.lost) {
        goto searchForBall;
      }
      if (!veryCloseToOpponentGoal() && !veryCloseToOwnGoal()) {
        goto decideNextAction;
      }
    }
    action {
      gloBallTargetAngle = libCodeRelease.computeBallDefendingDirection();
      InWalkKickTo(gloBallTargetAngle);
    }
  }

  state(corner) {
    transition {
      if (theBallModel.lost) {
        goto searchForBall;
      }
      if (!ballTooCloseToSide()) {
        goto decideNextAction;
      }
    }
    action {
      gloBallTargetAngle = (Vector2<>(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosCenterGoal) -
                            libWorldModel.ballPosition())
                             .angle();
      InWalkKickTo(gloBallTargetAngle);
    }
  }

  // for (int i = 1; i < TeamMateData::numOfPlayers; i++) {
  //   if ((theTeamMateData.isFullyActive[i]) &&
  //       ((theTeamMateData.robotPoses[i].translation.x - theRobotPose.translation.x) >= 1500.f)) {
  //     activeStrikerFarAway = true;
  //     break;
  //   }
  // }

  // initial_state(play) {
  //   transition {
  //     if (theBallModel.lost) {
  //       goto searchForBall;
  //     }
  //     if (gloBallPos.x < theFieldDimensions.centerCircleRadius) {
  //       if (activeStrikerFarAway) {
  //         goto pass;
  //       } else {
  //         goto dribble;
  //       }
  //     }
  //     if ((gloBallPos - gloOpponentGoal).abs() > theFieldDimensions.xPosOpponentGroundline) {
  //       goto dribble;
  //     }
  //   }
  //   action {
  //     float nearestPlayerInSight = libCodeRelease.nearestPlayerInSight(0.8, 500);
  //     // DribbleTo(gloOpponentGoal);
  //     if (shouldRush()) {
  //       // ScoreGoal takes care of rushing
  //       ScoreGoal();
  //     } else if (nearestPlayerInSight < 750.f) {
  //       DribbleTo(gloOpponentGoal);
  //     } else {
  //       ScoreGoal();
  //     }
  //   }
  // }

  // state(dribble) {
  //   transition {
  //     if (theBallModel.lost) {
  //       goto searchForBall;
  //     }
  //     if (gloBallPos.x < theFieldDimensions.centerCircleRadius) {
  //       if (activeStrikerFarAway) {
  //         goto pass;
  //       }
  //     }
  //     if ((gloBallPos - gloOpponentGoal).abs() < theFieldDimensions.xPosOpponentPenaltyArea) {
  //       goto play;
  //     }
  //   }
  //   action {
  //     if (relToGlo(theBallModel.estimate.position).y > 250.f) {
  //       DribbleTo(Vector2<>(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea));
  //     } else {
  //       DribbleTo(Vector2<>(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea));
  //     }
  //   }
  // }

  // // Only if at least one striker far away AND no other players detected near
  // // (basically, have a target to pass to & have time to align for power-kick)
  // state(pass) {
  //   transition {
  //     if (gloBallPos.x > 2 * theFieldDimensions.centerCircleRadius) {
  //       goto play;
  //     }
  //   }
  //   action {
  //     theHeadControlMode = HeadControl::lookAtBall;
  //     Vector2<> gloPassTarget;
  //     Vector2<> furthestStriker;
  //     float furthestStrikerDistance = -1.f;

  //     for (int i = 1; i < TeamMateData::numOfPlayers; i++) {
  //       if ((theTeamMateData.isFullyActive[i]) &&
  //           ((theTeamMateData.robotPoses[i].translation.x - theRobotPose.translation.x) >= furthestStrikerDistance)) {
  //         furthestStrikerDistance = theTeamMateData.robotPoses[i].translation.x - theRobotPose.translation.x;
  //         furthestStriker = theTeamMateData.robotPoses[i].translation;
  //       }
  //     }

  //     if (furthestStrikerDistance > 0.f) {
  //       KickTo(furthestStriker);
  //     } else {
  //       KickTo(gloOpponentGoal);
  //     }
  //   }
  // }

  state(searchForBall) {
    transition {
      if (ballKickedAway) {
        ballKickedAway = false;
        goto travelToKickTarget;
      } else {
        goto localSearch;
      }
    }
    action {}
  }

  state(travelToKickTarget) {
    transition {
      if (theBallModel.valid) {
        goto decideNextAction;
      }
      if ((gloKickEstimate - theRobotPose.translation).abs() < 1000.f) {
        goto localSearch;
      }
    }
    action { TravelTo(Pose2D((gloKickEstimate - theRobotPose.translation).angle(), gloKickEstimate.x, gloKickEstimate.y)); }
  }

  state(localSearch) {
    transition {
      if (theBallModel.valid) {
        goto decideNextAction;
      }
      if (state_time > 8000) {
        goto ballLost;
      }
    }
    action { WalkAtSpeed(Pose2D(1.5f, 0.f, 0.f)); }
  }

  target_state(ballScored) {}

  aborted_state(ballLost) {}
}

bool veryCloseToOpponentGoal() {
  const Vector2<> goalTarget = Vector2<>(theFieldDimensions.xPosOpponentGroundline - 100, 0.f);
  return (theRobotPose * theBallModel.estimate.position - goalTarget).abs() < theFieldDimensions.yPosLeftGoalBox * 1.5f;
}

bool veryCloseToOwnGoal() {
  const Vector2<> goalTarget = Vector2<>(theFieldDimensions.xPosOwnGroundline + 100, 0.f);
  return (theRobotPose * theBallModel.estimate.position - goalTarget).abs() < theFieldDimensions.yPosLeftGoalBox * 1.5f;
}

bool ballTooCloseToSide() {
  return (libWorldModel.ballPosition().x > theFieldDimensions.xPosOpponentGoalBox &&
          std::abs(libWorldModel.ballPosition().y) > theFieldDimensions.yPosLeftPenaltyArea);
}