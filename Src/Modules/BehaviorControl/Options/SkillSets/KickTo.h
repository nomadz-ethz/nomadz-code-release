/**
 * @file KickTo.h
 *
 * Walk to the ball and kick it towards some position on the field, as quickly as possible.
 * On the way to the ball, walks around other players to avoid collision.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(KickTo, Vector2<> gloBallTargetPos) {

  Vector2<> preKickBallPositionEstimate;

  Vector2<> gloBallPos = theRobotPose * theBallModel.estimate.position;
  Angle gloRobot2BallAngle = (gloBallPos - theRobotPose.translation).angle();
  Angle gloBallTargetAngle = (gloBallTargetPos - gloBallPos).angle();
  Angle relBallTargetAngle = libCodeRelease.computeRelBallTargetAngle(gloBallTargetAngle);

  initial_state(getBehindBall) {
    transition {
      if (action_done) {
        goto accurateAlignment;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      BallAlignment(gloBallTargetAngle);
    }
  }

  state(accurateAlignment) {
    transition {
      if (action_done) {
        preKickBallPositionEstimate = theBallModel.estimate.position;
        goto kick;
      }
      if (!libCodeRelease.insidePatternCone(theBallModel.estimate.position, gloBallTargetAngle)) {
        goto getBehindBall;
      }
    }
    action { PatternAlignment(relBallTargetAngle); }
  }

  state(kick) {
    transition {
      if ((theBallModel.estimate.position - preKickBallPositionEstimate).abs() > 200 &&
          theKickEngineOutput.isLeavingPossible) {
        goto getBehindBall;
      }
      if (state_time > 3000) {
        goto done;
      }
    }
    action {
      const Vector2<> powerKickOffset = Vector2<>(3000.f, 0.f);
      gloKickEstimate = theRobotPose * powerKickOffset;
      ballKickedAway = true;
      PowerKick();
    }
  }

  state(kickPostBreak) {
    transition {
      if (state_time > 500) {
        goto done;
      }
    }
    action { StandWide(); }
  }

  target_state(done) {
    transition {
      if (state_time > 500) {
        goto getBehindBall;
      }
    }
    action { Stand(); }
  }
}
