/**
 * @file WalkingEngine.cpp
 *
 * This file implements a module that is a wrapper for the UNSW walk generator.
 *
 * This file is subject to the terms of the BHuman 2017 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#include "WalkingEngine.h"

MAKE_MODULE(WalkingEngine, Motion Control);

WalkingEngine::WalkingEngine() {
  theWalkGeneratorData.reset();
  thePlannedSteps.reset();
}

void WalkingEngine::update(WalkingEngineOutput& walkingEngineOutput) {

  // Check if engine is active
  if (theMotionSelection.ratios[MotionRequest::walk] > 0.f) {
    // Generate the walk request
    walkRequest = theMotionSelection.walkRequest;
    if (theMotionRequest.motion == MotionRequest::walk && theMotionSelection.ratios[MotionRequest::walk] == 1.f &&
        theGroundContactState.contact &&
        (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering)) {
      // TODO [walk] handle in walk kick
      lastTimeWalking = theFrameInfo.time;
      Pose2D maxSpeed;
      switch (walkRequest.mode) {
      case WalkRequest::patternMode:
        walkMode = WalkGeneratorData::WalkMode::patternMode;
        speed = walkRequest.speed == Pose2D() ? Pose2D(0.000001f, 0.f) : walkRequest.speed;
        break;

      case WalkRequest::speedMode:
        speed = walkRequest.speed == Pose2D() ? Pose2D(0.000001f, 0.f) : walkRequest.speed;
        walkMode = WalkGeneratorData::speedMode;
        break;

      case WalkRequest::targetMode:
        maxSpeed.rotation = std::max(walkRequest.speed.rotation, 0.8f);
        maxSpeed.translation.x = std::max(walkRequest.speed.translation.x, 0.2f);
        maxSpeed.translation.y = std::max(walkRequest.speed.translation.y, 0.8f);
        speed = maxSpeed.elementwiseMul(theWalkGeneratorData.maxSpeed);
        target = walkRequest.target;
        walkMode = WalkGeneratorData::targetMode;
        break;

      default:
        speed = Pose2D();
        walkMode = WalkGeneratorData::speedMode;
        break;
      }

    } else {
      speed = Pose2D();
      walkMode = WalkGeneratorData::speedMode;
    }

    bool stepStarted = theWalkGeneratorData.t == 0;
    theWalkGeneratorData.calcJoints(speed, target, walkMode);
    jointRequest = theWalkGeneratorData.jointRequest;
    updateOutput(stepStarted, walkingEngineOutput);
  } else {
    walkingEngineOutput.standing = true;
    walkingEngineOutput.odometryOffset = Pose2D();
    walkingEngineOutput.speed = Pose2D();
    walkingEngineOutput.isLeavingPossible = true;
    walkingEngineOutput.maxSpeed = theWalkGeneratorData.maxSpeed;

    theWalkGeneratorData.reset();
    thePlannedSteps.reset();

    jointRequest.jointAngles.angles[JointData::FirstArmJoint] = JointData::off;
  }
}

void WalkingEngine::updateOutput(bool stepStarted, WalkingEngineOutput& walkingEngineOutput) {
  if (stepStarted) {
    walkingEngineOutput.speed = theWalkGeneratorData.speed;
    walkingEngineOutput.upcomingOdometryOffset = theWalkGeneratorData.upcomingOdometryOffset;
    walkingEngineOutput.executedWalk = walkRequest;
    walkingEngineOutput.standing = walkingEngineOutput.speed == Pose2D() && walkingEngineOutput.odometryOffset == Pose2D();
  }

  walkingEngineOutput.isLeavingPossible =
    theFrameInfo.getTimeSince(lastTimeWalking) >= minTimeInStandBeforeLeaving &&
    std::abs(theRobotModel.soleLeft.translation.z - theRobotModel.soleRight.translation.z) < 5.f;

  walkingEngineOutput.odometryOffset = theWalkGeneratorData.odometryOffset;
  walkingEngineOutput.upcomingOdometryOffset -= walkingEngineOutput.odometryOffset;
  walkingEngineOutput.maxSpeed = theWalkGeneratorData.maxSpeed;
  walkingEngineOutput.requestedLeftFoot = theWalkGeneratorData.stepTraj[Leg::left].footFinal;
  walkingEngineOutput.requestedRightFoot = theWalkGeneratorData.stepTraj[Leg::right].footFinal;
  // TODO [walk] integrate odometry offset into target
  // Pose2D odometryOffset = theWalkGeneratorData.odometryOffset;
  // odometryOffset.rotation = Angle::normalize(-theOrientationData.rotation.getZAngle() - theOdometryData.rotation);
  // target -= odometryOffset;

  for (int i = 0; i < JointData::numOfJoints; ++i) {
    walkingEngineOutput.jointRequest.jointAngles.angles[i] = jointRequest.jointAngles.angles[i];

    walkingEngineOutput.jointRequest.jointHardness.hardness[i] = jointRequest.jointHardness.hardness[i];
  }
}
