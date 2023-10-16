/**
 * @file TeamDataSender.cpp
 *
 * Implementation of module TeamDataSender
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
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

    const auto packagesToSendSelector =
      static_cast<TeamMateDataSelector>(static_cast<unsigned char>(theTeamMateData.packagesToSend));
    // Send certain outputs only in specific states
    if (theGameInfo.state == STATE_SET) {
      TEAM_OUTPUT(idWhistle, bin, theWhistle);
    } else if (theGameInfo.state == STATE_PLAYING) {

      // Pose, ball and obstacles
      if (toIntegral(packagesToSendSelector & TeamMateDataSelector::robotPose)) {
        TEAM_OUTPUT(idTeamMateRobotPose, bin, RobotPoseCompressed(theRobotPose));
      }

      if (toIntegral(packagesToSendSelector & TeamMateDataSelector::sideConfidence)) {
        TEAM_OUTPUT(idTeamMateSideConfidence, bin, theSideConfidence);
      }

      if (toIntegral(packagesToSendSelector & TeamMateDataSelector::ballModel)) {
        TEAM_OUTPUT(idTeamMateBallModel, bin, BallModelCompressed(theBallModel));
      }
      // FIXME: Send player model when there is space.
      if (toIntegral(packagesToSendSelector & TeamMateDataSelector::playerModel)) {
        TEAM_OUTPUT(idTeamMatePlayerModel, bin, PlayerModelCompressed(thePlayerModel, 0 /* SEND NO PLAYERS */));
      }

      // User Defined Data
      if (toIntegral(packagesToSendSelector & TeamMateDataSelector::personalData)) {
        TEAM_OUTPUT(idPersonalData, bin, PersonalDataCompressed(thePersonalData));
      }
    }

    // Robot status
    if (toIntegral(packagesToSendSelector & TeamMateDataSelector::fallDownState)) {

      TEAM_OUTPUT(idTeamMateIsUpright, bin, static_cast<char>(theFallDownState.state == theFallDownState.upright));

      TEAM_OUTPUT(idTeamMateHasGroundContact, bin, static_cast<char>(theGroundContactState.contact));
      if (theGroundContactState.contact)
        TEAM_OUTPUT(idTeamMateTimeSinceLastGroundContact, bin, theFrameInfo.time);
    }

    // Behavior
    if (toIntegral(packagesToSendSelector & TeamMateDataSelector::behaviorStatus)) {
      TEAM_OUTPUT(idTeamMateBehaviorStatus, bin, theBehaviorStatus);
    }
  }
}