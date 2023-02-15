/**
 * @file TeamDataSender.cpp
 *
 * Implementation of module TeamDataSender
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are Colin Graf
 */

#include "TeamDataSender.h"
#include "Tools/Team.h"
#include "Core/System/Time.h"

MAKE_MODULE(TeamDataSender, Cognition Infrastructure)

void TeamDataSender::update(TeamDataSenderOutput& teamDataSenderOutput) {
  if (theTeamMateData.sendThisFrame) {
    ++sendFrames;
    Global::getStdMsg().playerNum = (uint8_t)theRobotInfo.number;
    Global::getStdMsg().teamNum = (int8_t)theOwnTeamInfo.teamNumber;
    Global::getStdMsg().fallen = (uint8_t)(theFallDownState.state != FallDownState::upright);
    Global::getStdMsg().pose[0] = (float)theRobotPose.translation.x;
    Global::getStdMsg().pose[1] = (float)theRobotPose.translation.y;
    Global::getStdMsg().pose[2] = (float)normalize(theRobotPose.rotation);
    if (theBallModel.timeSinceLastSeen == INT_MAX) {
      Global::getStdMsg().ballAge = -1.f;
    } else {
      Global::getStdMsg().ballAge = theBallModel.timeSinceLastSeen / 1000.0;
    }
    Global::getStdMsg().ball[0] = (float)theBallModel.estimate.position.x;
    Global::getStdMsg().ball[1] = (float)theBallModel.estimate.position.y;

    // User Defined Data - Sent
    TEAM_OUTPUT(idPersonalData, bin, thePersonalData);

    // Own pose information and ball observation:
    TEAM_OUTPUT(idTeamMateRobotPose, bin, RobotPoseCompressed(theRobotPose));
    TEAM_OUTPUT(idTeamMateSideConfidence, bin, theSideConfidence);
    TEAM_OUTPUT(idTeamMateBallModel, bin, BallModelCompressed(theBallModel));

    // Obstacle
    TEAM_OUTPUT(idTeamMatePlayerModel, bin, PlayerModelCompressed(thePlayerModel, maxNumberOfRobotsToSend));

    // Robot status
    TEAM_OUTPUT(idTeamMateHasGroundContact, bin, theGroundContactState.contact);
    TEAM_OUTPUT(idTeamMateIsUpright, bin, (theFallDownState.state == theFallDownState.upright));
    if (theGroundContactState.contact)
      TEAM_OUTPUT(idTeamMateTimeSinceLastGroundContact, bin, theFrameInfo.time);
    TEAM_OUTPUT(idWhistle, bin, theWhistle);

    // Behavior
    TEAM_OUTPUT(idTeamMateBehaviorStatus, bin, theBehaviorControlOutput.behaviorStatus);
  }
}
