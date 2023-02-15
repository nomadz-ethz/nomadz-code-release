/**
 * @file ArmAnimator.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "ArmAnimator.h"
#include "Core/Settings.h"

ArmAnimator::ArmAnimator() {
  lastContactStateLeft = ArmContact::ArmContactState::None;
  lastContactStateRight = ArmContact::ArmContactState::None;
  leftTimeWhenNoContact = 0;
  rightTimeWhenNoContact = 0;
  timeWhenChangedToWalk = 0;
}

ArmAnimator::~ArmAnimator() {}

void ArmAnimator::update(ArmMovement& armMovement) {
  WalkingEngineParams curparams = (WalkingEngineParams&)theWalkingEngineParams;

  armMovement.jointAngles.angles[JointData::LShoulderPitch] = 0;
  armMovement.jointAngles.angles[JointData::LShoulderRoll] = Angle::fromDegrees(curparams.arms1);
  armMovement.jointAngles.angles[JointData::LElbowRoll] = 0;
  armMovement.jointAngles.angles[JointData::LElbowYaw] = pi_2;
  // BEMBLECUP Changed Wrist Direction from wristOffset to -wristOffset
  armMovement.jointAngles.angles[JointData::LWristYaw] = -wristOffset;
  armMovement.jointAngles.angles[JointData::LHand] = handOffset;

  armMovement.jointAngles.angles[JointData::RShoulderPitch] = 0;
  armMovement.jointAngles.angles[JointData::RShoulderRoll] = Angle::fromDegrees(-curparams.arms1);
  armMovement.jointAngles.angles[JointData::RElbowRoll] = 0;
  armMovement.jointAngles.angles[JointData::RElbowYaw] = -pi_2;
  armMovement.jointAngles.angles[JointData::RWristYaw] = wristOffset;
  armMovement.jointAngles.angles[JointData::RHand] = handOffset;

  float xOffset = (theKinematicRequest.leftFoot[0] + theKinematicRequest.rightFoot[0]) / 2;
  float leftArm, rightArm;

  leftArm = Angle::fromDegrees(90) + curparams.armFactor * (theKinematicRequest.leftFoot[0] - xOffset);
  rightArm = Angle::fromDegrees(90) + curparams.armFactor * (theKinematicRequest.rightFoot[0] - xOffset);
  leftArm = leftArm < 0 ? 0 : (leftArm > pi ? pi : leftArm);
  rightArm = rightArm < 0 ? 0 : (rightArm > pi ? pi : rightArm);

  if (std::fabs(theBodyTilt.y) < theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1] &&
      std::fabs(theBodyTilt.y) > theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0]) {
    armMovement.jointAngles.angles[JointData::LShoulderPitch] = leftArm - theBodyTilt.y;
    armMovement.jointAngles.angles[JointData::RShoulderPitch] = rightArm - theBodyTilt.y;
  }

  armMovement.usearms = true;
  armMovement.armsInContactAvoidance = false;

  // arm contact stuff

  // Compute times
  const int timeToMoveArm = timeToPullPitch + timeToPullArmIn;
  const int time4_5 = timeToPullPitch - timeToPullPitch / 5;
  {
    // set timestamps for arm movement start/end based on contact state
    // ..for left arm
    if (theMotionSelection.targetMotion == MotionRequest::walk && theMotionSelection.ratios[MotionRequest::walk] < 1.f) {
      timeWhenChangedToWalk = theFrameInfo.time;
    }

    if (theMotionRequest.motion == MotionRequest::walk && theMotionSelection.ratios[MotionRequest::walk] == 1.f &&
        theFrameInfo.getTimeSince(timeWhenChangedToWalk) > timeUntilArmContactActive) {
      if (theArmContact.armContactStateLeft != lastContactStateLeft) {
        if (theArmContact.armContactStateLeft != ArmContact::ArmContactState::None &&
            (theFrameInfo.time > leftTimeWhenNoContact || leftTimeWhenNoContact == 0)) {
          leftPitchWhenContactStateChanged = theJointData.angles[JointData::LShoulderPitch];
          leftTimeWhenContact = theFrameInfo.time;
          leftTimeWhenNoContact = theFrameInfo.time + timeToHoldArmBack;
        } else if (theArmContact.armContactStateLeft == ArmContact::ArmContactState::None) {
        }
      }
      if (theArmContact.armContactStateLeft != ArmContact::ArmContactState::None) {
        leftTimeWhenNoContact = theFrameInfo.time + timeToHoldArmBack;
      }

      // ..for right arm
      if (theArmContact.armContactStateRight != lastContactStateRight) {
        if (theArmContact.armContactStateRight != ArmContact::ArmContactState::None &&
            (theFrameInfo.time > rightTimeWhenNoContact || rightTimeWhenNoContact == 0)) {
          rightPitchWhenContactStateChanged = theJointData.angles[JointData::RShoulderPitch];
          rightTimeWhenContact = theFrameInfo.time;
          rightTimeWhenNoContact = theFrameInfo.time + timeToHoldArmBack;
        } else if (theArmContact.armContactStateLeft == ArmContact::ArmContactState::None) {
        }
      }
      if (theArmContact.armContactStateRight != ArmContact::ArmContactState::None) {
        rightTimeWhenNoContact = theFrameInfo.time + timeToHoldArmBack;
      }

      lastContactStateLeft = theArmContact.armContactStateLeft;
      lastContactStateRight = theArmContact.armContactStateRight;
    }

    if (lastMotion == MotionRequest::walk && theMotionRequest.motion != MotionRequest::walk) {
      rightTimeWhenNoContact = std::min(theFrameInfo.time - timeToMoveArm, rightTimeWhenNoContact);
      leftTimeWhenNoContact = std::min(theFrameInfo.time - timeToMoveArm, leftTimeWhenNoContact);
      lastContactStateLeft = theArmContact.armContactStateLeft;
      lastContactStateRight = theArmContact.armContactStateRight;
      timeWhenChangedToWalk = theFrameInfo.time; // to prevent motion request switches activating/deactiving arm movement
      lastMotion = theMotionRequest.motion;
      return;
    }

    // arm movement
    // Left Arm
    int timeSinceContact = theFrameInfo.getTimeSince(leftTimeWhenContact);
    int timeSinceNoContact =
      (leftTimeWhenNoContact > theFrameInfo.time) ? -1 : theFrameInfo.getTimeSince(leftTimeWhenNoContact);
    {
      // back to original state
      if (timeSinceNoContact >= 0 && timeSinceNoContact < timeToMoveArm) {
        float alpha = std::min((float)(timeSinceNoContact) / timeToPullArmIn, 1.0f);
        armMovement.jointAngles.angles[JointData::LElbowRoll] = (1 - alpha) * armBackElbowRoll;
        if (alpha < 1.f) {
          armMovement.jointAngles.angles[JointData::LShoulderRoll] =
            Angle::normalize((alpha)*armToFrontShoulderRoll + (1 - alpha) * -armBackShoulderRoll);
        }
        alpha = (timeSinceNoContact > timeToPullArmIn) ? (float)(timeSinceNoContact - timeToPullArmIn) / timeToPullPitch : 0;
        alpha = std::min(alpha, 1.f);
        // to get to an angle in constant time, take standard from walk (90 degree) as best guess
        armMovement.jointAngles.angles[JointData::LShoulderPitch] =
          Angle::normalize((alpha)*Angle::fromDegrees(90) + (1 - alpha) * armBackPitch);
        if (alpha > 0) {
          armMovement.jointAngles.angles[JointData::LShoulderRoll] =
            Angle::normalize((alpha)*Angle::fromDegrees(curparams.arms1) + (1 - alpha) * armToFrontShoulderRoll);
        }
      }
      // pull in
      else if (timeSinceContact >= 0 && timeSinceContact < (int)leftTimeWhenNoContact - (int)leftTimeWhenContact) {
        float alpha = std::min((float)timeSinceContact / timeToPullPitch, 1.0f);
        armMovement.jointAngles.angles[JointData::LShoulderPitch] =
          Angle::normalize((1 - alpha) * leftPitchWhenContactStateChanged + alpha * armBackPitch);
        alpha = (timeSinceContact > time4_5) ? (float)(timeSinceContact - time4_5) / timeToPullArmIn : 0;
        alpha = std::min(alpha, 1.f);
        armMovement.jointAngles.angles[JointData::LElbowRoll] = alpha * armBackElbowRoll;
        armMovement.jointAngles.angles[JointData::LShoulderRoll] =
          Angle::normalize((1 - alpha) * Angle::fromDegrees(curparams.arms1) + (alpha) * -armBackShoulderRoll);
      }
    }

    // Right Arm
    timeSinceContact = theFrameInfo.getTimeSince(rightTimeWhenContact);
    timeSinceNoContact =
      (rightTimeWhenNoContact > theFrameInfo.time) ? -1 : theFrameInfo.getTimeSince(rightTimeWhenNoContact);
    {
      // back to original state
      if (timeSinceNoContact >= 0 && timeSinceNoContact < timeToMoveArm) {
        float alpha = std::min((float)(timeSinceNoContact) / timeToPullArmIn, 1.0f);
        armMovement.jointAngles.angles[JointData::RElbowRoll] = (1 - alpha) * (-armBackElbowRoll);
        if (alpha < 1.f) {
          armMovement.jointAngles.angles[JointData::RShoulderRoll] =
            Angle::normalize((alpha) * -armToFrontShoulderRoll + (1 - alpha) * armBackShoulderRoll);
        }
        alpha = (timeSinceNoContact > timeToPullArmIn) ? (float)(timeSinceNoContact - timeToPullArmIn) / timeToPullPitch : 0;
        alpha = std::min(alpha, 1.f);
        // to get to an angle in constant time, take standard from walk (90 degree) as best guess
        armMovement.jointAngles.angles[JointData::RShoulderPitch] =
          Angle::normalize((alpha)*Angle::fromDegrees(90) + (1 - alpha) * armBackPitch);
        if (alpha > 0) {
          armMovement.jointAngles.angles[JointData::RShoulderRoll] =
            Angle::normalize((alpha)*Angle::fromDegrees(-curparams.arms1) + (1 - alpha) * -armToFrontShoulderRoll);
        }
      }
      // pull in
      else if (timeSinceContact >= 0 && timeSinceContact < (int)rightTimeWhenNoContact - (int)rightTimeWhenContact) {
        float alpha = std::min((float)timeSinceContact / timeToPullPitch, 1.0f);
        armMovement.jointAngles.angles[JointData::RShoulderPitch] =
          Angle::normalize((1 - alpha) * rightPitchWhenContactStateChanged + alpha * armBackPitch);
        alpha = (timeSinceContact > time4_5) ? (float)(timeSinceContact - time4_5) / timeToPullArmIn : 0;
        alpha = std::min(alpha, 1.f);
        armMovement.jointAngles.angles[JointData::RElbowRoll] = alpha * (-armBackElbowRoll);
        armMovement.jointAngles.angles[JointData::RShoulderRoll] =
          Angle::normalize((1 - alpha) * Angle::fromDegrees(-curparams.arms1) + (alpha)*armBackShoulderRoll);
      }
    }
  }

  if (leftTimeWhenContact < theFrameInfo.time && leftTimeWhenNoContact + timeToMoveArm > theFrameInfo.time) {
    armMovement.armsInContactAvoidance = true;
  }
  if (rightTimeWhenContact < theFrameInfo.time && rightTimeWhenNoContact + timeToMoveArm > theFrameInfo.time) {
    armMovement.armsInContactAvoidance = true;
  }

  lastMotion = theMotionRequest.motion;
}
MAKE_MODULE(ArmAnimator, dortmundWalkingEngine)
