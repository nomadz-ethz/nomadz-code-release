/**
 * @file HandleUncertainty.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

bool shouldEngageBall = false;
bool freeKick = false;

option(HandleUncertainty) {
  Angle ballAlignmentAngle = libCodeRelease.computeAlignmentAngle(theBallModel.estimate.position);
  bool faceTheBallCondition =
    !thePersonalData.hasBallLock && theBallModel.valid && std::abs(ballAlignmentAngle) > fromDegrees(45.f);
  // BallFree summarizes the conditions from the freekick and help the player decide when it can engage the ball.
  freeKick = theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK ||
             theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN;
  bool ballFree = (theGameInfo.setPlay == SET_PLAY_NONE || (freeKick && isKickingTeam));
  shouldEngageBall = thePersonalData.hasBallLock && ballFree;

  initial_state(keyObservationInvalid) {
    transition {
      if (faceTheBallCondition) {
        goto faceTheBall;
      }
      if ((theBallModel.valid || theCombinedWorldModel.ballIsValidOthers) && theRobotPose.valid) {
        goto keyObservationValid;
      }
    }
    action { SearchAndRelocate(); }
  }

  state(keyObservationValid) {
    transition {
      if ((theBallModel.lost && !theCombinedWorldModel.ballIsValidOthers) || theRobotPose.lost) {
        if (!thePersonalData.hasBallLock) {
          goto keyObservationInvalid;
        }
      }
    }
    action { ActiveRoleGenericBehavior(); }
  }

  state(faceTheBall) {
    transition {
      if (theBallModel.lost) {
        goto keyObservationInvalid;
      }
      if (std::abs(ballAlignmentAngle) < fromDegrees(15.f)) {
        if (theRobotPose.lost) {
          goto keyObservationInvalid;
        } else {
          goto keyObservationValid;
        }
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2D(1.f, 0.f, 0.f), Pose2D(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }
}