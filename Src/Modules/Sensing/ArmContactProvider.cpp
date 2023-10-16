/**
 * @file ArmContactProvider.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2023 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:dino.menges@tu-dortmund.de> Dino Menges</a>
 */

#include "ArmContactProvider.h"
#include "Core/Debugging/DebugDrawings.h"

Vector2<> gloToRel(const Pose2D& robotPose, const Vector2<>& target) {
  Vector2<> ret;
  float dist = float(sqrt(pow(target[0] - robotPose.translation.x, 2) + pow(target[1] - robotPose.translation.y, 2)));
  float alpha = robotPose.rotation - atan2f(target[1] - robotPose.translation.y, target[0] - robotPose.translation.x);

  ret[0] = dist * cosf(alpha);
  ret[1] = -dist * sinf(alpha);
  return ret;
}

ArmContactProvider::ArmContactProvider() {
  localArmContact.armContactStateLeft = ArmContact::None;
  localArmContact.armContactStateRight = ArmContact::None;
  localArmContact.timeStampLeft = 0;
  localArmContact.timeStampRight = 0;
  resetBuffers();
  lastPitchLeft = lastPitchRight = 0;
  lastRequest.jointAngles.timeStamp = 0;
  falling = false;
  timeWhenWalkStarted = 0;
  lastMotionType = MotionRequest::specialAction;
}

ArmContactProvider::~ArmContactProvider() {}

void ArmContactProvider::update(ArmContact& armContact) {
  // Check if lastRequest has been set
  if (lastRequest.jointAngles.timeStamp == 0) {
    lastRequest = theJointRequest;
  }

  // just check if it is in walk or not
  if (lastMotionType != MotionRequest::walk && theMotionSelection.ratios[MotionRequest::walk] == 1.f) {
    lastMotionType = MotionRequest::walk;
    timeWhenWalkStarted = theFrameInfo.time;
  } else if (theMotionSelection.ratios[MotionRequest::walk] != 1.f) {
    lastMotionType = MotionRequest::specialAction; // whatever
  }

  // Only when in standard walk (not in kicking, not in special actions etc)
  if (enableAvoidArmContact && (theGameInfo.state == STATE_READY || theGameInfo.state == STATE_PLAYING) &&
      lastMotionType == MotionRequest::walk &&
      theFrameInfo.getTimeSince(timeWhenWalkStarted) >
        2000 && // a little time to get started here to not move arms immediately on walk start
      theFallDownState.state == FallDownState::upright) {
    localArmContact.armContactStateLeft = ArmContact::ArmContactState::None;
    localArmContact.armContactStateRight = ArmContact::ArmContactState::None;

    bufferLeft.push_front(0);
    bufferRight.push_front(0);

    bool obstacleLeft = false;
    bool obstacleRight = false;

    // avoid contact with goal posts
    Vector2<> goalPostRight(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
    Vector2<> goalPostLeft(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
    // Vector2<> goalPostRightRelative = Transformation::fieldToRobot(theRobotPose, goalPostRight);
    // Vector2<> goalPostLeftRelative = Transformation::fieldToRobot(theRobotPose, goalPostLeft);
    Vector2<> goalPostRightRelative = gloToRel(theRobotPose, goalPostRight);
    Vector2<> goalPostLeftRelative = gloToRel(theRobotPose, goalPostLeft);
    if (goalPostRightRelative.abs() < 300 && goalPostRightRelative.angle() < 0) {
      obstacleRight = true;
    }
    if (goalPostLeftRelative.abs() < 300 && goalPostLeftRelative.angle() > 0) {
      obstacleLeft = true;
    }

    goalPostRight = Vector2<>(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
    goalPostLeft = Vector2<>(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
    // goalPostRightRelative = Transformation::fieldToRobot(theRobotPose, goalPostRight);
    // goalPostLeftRelative = Transformation::fieldToRobot(theRobotPose, goalPostLeft);
    goalPostRightRelative = gloToRel(theRobotPose, goalPostRight);
    goalPostLeftRelative = gloToRel(theRobotPose, goalPostLeft);
    if (goalPostRightRelative.abs() < 300 && goalPostRightRelative.angle() < 0) {
      obstacleRight = true;
    }
    if (goalPostLeftRelative.abs() < 300 && goalPostLeftRelative.angle() > 0) {
      obstacleLeft = true;
    }

    // avoid contact via robot map

    //    if (useRobotMap)
    //    {
    //      for (auto &robot : theRobotMap.robots)
    //      {
    //        Vector2<> robotRelative = Transformation::fieldToRobot(theRobotPose, robot.pose.translation);
    //        float robotDist = robotRelative.norm();
    //        //float ballDist = theBallModelAfterPreview.estimate.position.norm();
    //        if (std::abs(toDegrees(robotRelative.angle())) < 90 &&
    //          robotDist < maxRobotDist)
    //        {
    //          obstacleLeft = obstacleLeft || robotRelative.angle() > 0;
    //          obstacleRight = obstacleRight || robotRelative.angle() < 0;
    //        }
    //      }
    //    }

    if (useArmPitchDiff) {
      // set arm contact state and time stamp of arm contact using pitch differences
      if (bufferLeft.sum() > maxSum) {
        localArmContact.armContactStateLeft = ArmContact::Front;
      } else if (bufferLeft.sum() < -maxSum) {
        localArmContact.armContactStateLeft = ArmContact::Back;
      }

      if (bufferRight.sum() > maxSum) {
        localArmContact.armContactStateRight = ArmContact::Front;
      } else if (bufferRight.sum() < -maxSum) {
        localArmContact.armContactStateRight = ArmContact::Back;
      }
    }
    if (obstacleLeft) {
      localArmContact.armContactStateLeft = ArmContact::Front;
    }
    if (obstacleRight) {
      localArmContact.armContactStateRight = ArmContact::Front;
    }
  } else // No walk motion
  {
    localArmContact.armContactStateLeft = ArmContact::None;
    localArmContact.armContactStateRight = ArmContact::None;
    localArmContact.timeStampLeft = 0;
    localArmContact.timeStampRight = 0;

    bufferLeft.push_front(0);
    bufferRight.push_front(0);
  }

  // if robot might fall down, put arm back to side
  float fallDownAngle = falling ? 0.25f : 0.35f;
  falling = (std::abs(theInertiaSensorData.angle.y) > fallDownAngle);

  if (falling) {
    localArmContact.armContactStateLeft = ArmContact::None;
    localArmContact.armContactStateRight = ArmContact::None;
  }

  PLOT("module:ArmContactProvider:bufferLeft", bufferLeft.sum());
  PLOT("module:ArmContactProvider:bufferRight", bufferRight.sum());

  lastPitchLeft = theJointRequest.jointAngles.angles[JointData::LShoulderPitch];
  lastPitchRight = theJointRequest.jointAngles.angles[JointData::RShoulderPitch];

  ArmContact::ArmContactState leftState = armContact.armContactStateLeft;
  ArmContact::ArmContactState rightState = armContact.armContactStateRight;
  MODIFY("module:ArmContactProvider:leftState", leftState);
  MODIFY("module:ArmContactProvider:rightState", rightState);
  if (leftState != localArmContact.armContactStateLeft) {
    if (localArmContact.armContactStateLeft != ArmContact::None) {
      localArmContact.timeStampLeft = theFrameInfo.time;
      resetBuffers();
    }
  }
  if (rightState != localArmContact.armContactStateRight) {
    if (localArmContact.armContactStateRight != ArmContact::None) {
      localArmContact.timeStampRight = theFrameInfo.time;
      resetBuffers();
    }
  }

  lastRequest = theJointRequest;
  armContact = localArmContact;
}

void ArmContactProvider::resetBuffers() {
  bufferLeft.clear();
  bufferRight.clear();
}

MAKE_MODULE(ArmContactProvider, Motion Control)
