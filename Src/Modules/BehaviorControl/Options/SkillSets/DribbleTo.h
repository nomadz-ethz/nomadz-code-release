/**
 * @file DribbleTo.h
 *
 * Run behind ball, and if facing +/- 60 degrees towards opponent side, run into ball and do in walk kicks and shit.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(DribbleTo, Vector2<> gloBallTargetPos) {
  theHeadControlMode = HeadControl::lookForward;

  Vector2<> gloBallPos = theRobotPose * theBallModel.estimate.position;
  Angle gloRobot2BallAngle = (gloBallPos - theRobotPose.translation).angle();
  Angle gloBallTargetAngle = (gloBallTargetPos - gloBallPos).angle();
  Angle relBallTargetAngle = libCodeRelease.computeRelBallTargetAngle(gloBallTargetAngle);

  initial_state(decideNextAction) {
    transition {
      if (state_time > 1000) {
        if (std::abs(relBallTargetAngle) < fromDegrees(91.f)) {
          goto inWalkKickOmni;
        } else {
          goto ballAlignment;
        }
      }
    }
    action {
      WalkToTarget(Pose2D(1.f, 1.f, 0.f), Pose2D(theBallModel.estimate.position.angle(), theBallModel.estimate.position));
    }
  }

  state(ballAlignment) {
    transition {
      if (action_done) {
        goto inWalkKickStraight;
      }
    }
    action { BallAlignment(gloBallTargetAngle); }
  }

  state(inWalkKickStraight) {
    transition {
      if (action_done) {
        goto decideNextAction;
      }
      if (!libCodeRelease.insidePatternCone(theBallModel.estimate.position, gloBallTargetAngle)) {
        goto ballAlignment;
      }
    }
    action { PatternInwalkKick(relBallTargetAngle, 0.f); }
  }

  state(walkToBall) {
    transition {
      if (libCodeRelease.insidePatternCone(theBallModel.estimate.position, gloBallTargetAngle, 0.9)) {
        goto inWalkKickOmni;
      }
    }
    action { TravelTo(Pose2D(gloBallTargetAngle, gloBallPos)); }
  }

  state(inWalkKickOmni) {
    transition {
      if (action_done) {
        goto decideNextAction;
      }
      if (!libCodeRelease.insidePatternCone(theBallModel.estimate.position, gloBallTargetAngle)) {
        goto walkToBall;
      }
    }
    action { PatternInwalkKick(relBallTargetAngle, 0.f); }
  }
}
