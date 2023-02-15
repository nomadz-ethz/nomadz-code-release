/**
 * @file CSConverterModule.cpp
 *
 * This file implements a module that is a wrapper for the UNSW walk generator.
 *
 * This file is subject to the terms of the BHuman 2017 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
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
  /*
  // TODO [walk] replace logic below with the state matchine
  beginFrame(theFrameInfo.time);
  execute(OptionInfos::getOption("Root"));
  endFrame();
  */

  // Check if engine is active
  if (theMotionSelection.ratios[MotionRequest::walk] > 0.f) {
    // Generate the walk request
    if (theMotionRequest.motion == MotionRequest::walk && theMotionSelection.ratios[MotionRequest::walk] == 1.f &&
        theGroundContactState.contact &&
        (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering)) {
      // TODO [walk] handle in walk kick
      walkRequest = theMotionSelection.walkRequest;
      lastTimeWalking = theFrameInfo.time;

      // Generate parameters for appropriate walking mode
      if (walkRequest.mode == WalkRequest::patternMode) {
        this->walkMode = WalkGeneratorData::WalkMode::patternMode;
      } else if (walkRequest.mode == WalkRequest::speedMode) {
        walk(walkRequest.speed);
      } else if (walkRequest.mode == WalkRequest::percentageSpeedMode) {
        walk(walkRequest.speed.elementwiseMul(theWalkGeneratorData.maxSpeed));
      } else if (!std::isnan(walkRequest.target.translation.x)) {
        Pose2D maxSpeed = walkRequest.speed;
        // TODO [walk] ensure appropriate max speed percentages
        maxSpeed.rotation = std::max(maxSpeed.rotation, 0.8f);
        maxSpeed.translation.x = std::max(maxSpeed.translation.x, 0.2f);
        maxSpeed.translation.y = std::max(maxSpeed.translation.y, 0.8f);

        walk(maxSpeed.elementwiseMul(theWalkGeneratorData.maxSpeed), walkRequest.target);
      } else {
        std::cout << "ERROR" << std::endl;
        stand();
      }
    } else {
      stand();
    }

    bool stepStarted = theWalkGeneratorData.t == 0;
    thePlannedSteps.calcStepPattern(speed, target, walkMode);
    theWalkGeneratorData.calcJoints(speed, target, walkMode);
    jointRequest = theWalkGeneratorData.jointRequest;
    updateOutput(stepStarted, const_cast<WalkingEngineOutput&>(theWalkingEngineOutput));
  } else {
    WalkingEngineOutput& w = const_cast<WalkingEngineOutput&>(theWalkingEngineOutput);
    w.standing = true;
    w.odometryOffset = Pose2D();
    w.speed = Pose2D();
    w.isLeavingPossible = true;
    w.maxSpeed = theWalkGeneratorData.maxSpeed;
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

  walkingEngineOutput.isLeavingPossible = theFrameInfo.getTimeSince(lastTimeWalking) >= minTimeInStandBeforeLeaving;

  walkingEngineOutput.odometryOffset = theWalkGeneratorData.odometryOffset;
  walkingEngineOutput.upcomingOdometryOffset -= walkingEngineOutput.odometryOffset;
  walkingEngineOutput.maxSpeed = theWalkGeneratorData.maxSpeed;

  // TODO [walk] integrate odometry offset into target
  // Pose2D odometryOffset = theWalkGeneratorData.odometryOffset;
  // odometryOffset.rotation = Angle::normalize(-theOrientationData.rotation.getZAngle() - theOdometryData.rotation);
  // target -= odometryOffset;

  for (int i = 0; i < JointData::numOfJoints; ++i) {
    walkingEngineOutput.jointRequest.jointAngles.angles[i] = jointRequest.jointAngles.angles[i];

    walkingEngineOutput.jointRequest.jointHardness.hardness[i] = jointRequest.jointHardness.hardness[i];
  }
}

void WalkingEngine::walk(const Pose2D& speed,
                         WalkGeneratorData::WalkMode walkMode,
                         const std::function<Pose3D(float)>& getKickFootOffset) {
  ASSERT(walkMode != WalkGeneratorData::targetMode);

  // Prevent standing
  this->speed = speed == Pose2D() ? Pose2D(0.000001f, 0.f) : speed;
  this->walkMode = walkMode;
  this->getKickFootOffset = getKickFootOffset;
  lastTarget = Pose2D(100000.f, 100000.f);
}

void WalkingEngine::walk(const Pose2D& speed, const Pose2D& target) {
  // Prevent standing
  this->speed = speed;
  if (target != lastTarget) {
    this->target = lastTarget = target;
  }
  this->walkMode = WalkGeneratorData::targetMode;
  this->getKickFootOffset = std::function<Pose3D(float)>();
}

void WalkingEngine::stand() {
  walkRequest = theMotionSelection.walkRequest;
  speed = Pose2D();
  this->walkMode = WalkGeneratorData::speedMode;
  this->getKickFootOffset = std::function<Pose3D(float)>();
  lastTarget = Pose2D(100000.f, 100000.f);
}

void WalkingEngine::updateWalkRequestWithoutKick() {
  // TODO [walk] include kick info
  // WalkRequest::WalkKickRequest walkKickRequest = walkRequest.walkKickRequest;
  // walkRequest = theMotionRequest.walkRequest;
  // walkRequest.walkKickRequest = walkKickRequest;
}
