
/**
 * @file RefereeChallenge.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team theHeadLimits.maxPan()
 */

#include <iostream>
#include <vector>

option(RefereeChallenge) {
  initial_state(LookAtRef) {
    int ref_side_sign = (refereeSide == "left") ? 1 : -1;
    ref_side_sign = (theGameInfo.firstHalf == true) ? ref_side_sign : -1 * ref_side_sign;
    const Vector2<> refPos(0, ref_side_sign * theFieldDimensions.yPosLeftSideline);
    lookTargetAngle = angleToGlobalTarget(refPos);
    const Vector2<> refereeDistance(0, ref_side_sign * theFieldDimensions.yPosLeftSideline);
    // Compute tilt angle based on robot height and ideal image center height
    lookTargetTilt = -atan((950 - 550) / (theRobotPose.translation - refereeDistance).abs());
    transition {
      if (ref_side_sign < 0 && theRobotPose.translation.y < -theFieldDimensions.yPosLeftSideline + 1500) {
        goto endRefereeChallenge;
      } else if (ref_side_sign > 0 && theRobotPose.translation.y > theFieldDimensions.yPosLeftSideline - 1500) {
        goto endRefereeChallenge;
      } else if (abs(lookTargetAngle) < theHeadLimits.maxPan() && state_time > 5000) {
        theRefereePercept.runModel = true;
        goto runModel;
      }
    }
    action {
      if (abs(lookTargetAngle) >= theHeadLimits.maxPan()) {
        WalkToTarget(Pose2D(1.f, 0.f, 0.f), Pose2D(lookTargetAngle, Vector2<>(0, 0)));
      } else {
        theHeadControlMode = HeadControl::lookAtTargetWithTilt;
        Stand();
      }
    }
  }
  state(runModel) {
    transition {
      if (state_time > 9000) {
        goto react;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtTargetWithTilt;
      Stand();
    }
  }

  state(react) {
    transition {
      // Prediction score is in range [0, 1]
      // Send signal to gamecontroller sooner for higher prediction scores (since first received counts)
      // Send latest possible signal (score = 0) after 3 seconds, then end challenge
      float pred_threshold = 1.0 - (1.0 / 3000.0) * state_time;
      if (theRefereePercept.signal > 0 && theRefereePercept.predScore >= pred_threshold) {
        OUTPUT_TEXT("PREDICTED: " << theRefereePercept.signal);
        theRefereePercept.gamecontrollerSignal = theRefereePercept.signal;
        theRefereePercept.lastTimeGameControllerSent = theFrameInfo.time;

        bool play_sound = true;
        bool mirror_action = false;
        VisualRefereeReact(theRefereePercept.signal, play_sound, mirror_action);
        goto endRefereeChallenge;
      }
      if (state_time > 3000) {
        goto endRefereeChallenge;
      }
    }
  }
  target_state(endRefereeChallenge) {
    action {
      // stop running referee perceptor
      theRefereePercept.runModel = false;
      theRefereePercept.preds.clear();
      theRefereePercept.signal = -1;
    }
  }
}
