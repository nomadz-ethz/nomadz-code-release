/**
 * @file TeamMateData.h
 *
 * Declaration of a class representing information about the teammates.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Representations/Modeling/PlayerModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Plan.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Infrastructure/PersonalData.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/team_mate_data.hpp"
#endif
STREAMABLE_DECLARE(TeamMateData)

/** @class Bitmask strongly typed enum for selecting which data to send in the team communication.
 */
enum class TeamMateDataSelector : unsigned char {
  none = 0,
  robotPose = 1,
  sideConfidence = 2,
  ballModel = 4,
  playerModel = 8,
  personalData = 16,
  fallDownState = 32,
  behaviorStatus = 64,
  all = 127
};

/**
 * @class TeamMateData
 * A class representing information about the teammates.
 * Note: does not contain the information of the own robot, only team-mates!
 */

STREAMABLE_ROS(TeamMateData, {
public:
  ENUM(PlayerNum, noPlayer, firstPlayer, player1 = firstPlayer, player2, player3, player4, player5, player6, player7);
  static const int numOfPlayers = numOfPlayerNums;

  /** drawing function for representation*/
  void draw() const,

    // TeamMateData::numOfPlayers = 6 at this moment (Mar 2014)

    FIELD_WRAPPER(unsigned int, 0, nomadz_msgs::msg::TeamMateData::current_timestamp, currentTimestamp),
    FIELD_WRAPPER(unsigned int,
                  31000,
                  nomadz_msgs::msg::TeamMateData::network_timeout,
                  networkTimeout), /**< The time without packets received after which a teammate is considered absent. */
    FIELD_WRAPPER(
      unsigned, 0, nomadz_msgs::msg::TeamMateData::num_of_connected_team_mates, numOfConnectedTeamMates), /**< The number of
                                     robots of which messages were received recently. _Not_ including this robot itself. */
    FIELD_WRAPPER(unsigned,
                  0,
                  nomadz_msgs::msg::TeamMateData::num_of_active_team_mates,
                  numOfActiveTeamMates), /**< numOfConnectedTeamMates minus all robots that are currently penalized. */
    FIELD_WRAPPER(unsigned,
                  0,
                  nomadz_msgs::msg::TeamMateData::num_of_fully_active_team_mates,
                  numOfFullyActiveTeamMates), /**< numOfActiveTeamMates minus all robots that are currently not upright or do
not have ground contact */
    FIELD_WRAPPER(unsigned,
                  numOfPlayers,
                  nomadz_msgs::msg::TeamMateData::first_team_mate,
                  firstTeamMate), /**< player number of first team mate */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::TeamMateData::send_this_frame,
                  sendThisFrame), /**< The team communication will be sent in this frame. */
    FIELD_WRAPPER(unsigned char,
                  static_cast<char>(TeamMateDataSelector::none),
                  nomadz_msgs::msg::TeamMateData::packages_to_send,
                  packagesToSend), /**< which packages should be sent in the frame*/
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::TeamMateData::was_connected,
                  wasConnected), /**< Whether we have been connected to a team mate. */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::TeamMateData::whistle_detected_by_team_mates,
                  whistleDetectedByTeamMates), /**< Whether we have been connected to a team mate. */
    FIELD_WRAPPER_DEFAULT(unsigned int[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::time_stamps,
                          timeStamps), /**< The ntp times when messages from different robots arrived. */
    FIELD_WRAPPER_DEFAULT(
      bool[numOfPlayers], nomadz_msgs::msg::TeamMateData::is_active, isActive), /**< true, if messages from the respective
                             team mate were recently received and the team mate is currently not penalized. */
    FIELD_WRAPPER_DEFAULT(bool[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::is_fully_active,
                          isFullyActive), /**< true, if 'isActive' and has not been fallen down and has ground contact */
    FIELD_WRAPPER_DEFAULT(BallModel[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::ball_models,
                          ballModels), /**< The last received ball model of each team mate. */
    FIELD_WRAPPER_DEFAULT(RobotPose[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::robot_poses,
                          robotPoses), /**< The last received robot pose of each team mate. */
    FIELD_WRAPPER_DEFAULT(PlayerModel[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::player_models,
                          playerModels), /**< The last received robots model of each team mate. */
    FIELD_WRAPPER_DEFAULT(SideConfidence[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::robots_side_confidence,
                          robotsSideConfidence), /**< The last received side confidence of each team mate. */
    FIELD_WRAPPER_DEFAULT(BehaviorStatus[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::behavior_status,
                          behaviorStatus), /**< The last received behavior status of each team mate. */
    FIELD_WRAPPER_DEFAULT(bool[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::is_penalized,
                          isPenalized), /**< Tells us if a teammate is penalized. */
    FIELD_WRAPPER_DEFAULT(bool[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::has_ground_contact,
                          hasGroundContact), /**< Tells us if a teammate has ground contact. */
    FIELD_WRAPPER_DEFAULT(bool[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::is_upright,
                          isUpright), /**< Tells us if a teammate is fallen down. */
    FIELD_WRAPPER_DEFAULT(
      TeamHeadControlState[numOfPlayers], nomadz_msgs::msg::TeamMateData::team_head_control_states, teamHeadControlStates),
    FIELD_WRAPPER_DEFAULT(unsigned int[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::time_last_ground_contact,
                          timeLastGroundContact), /**< The time since last ground contact of a team mate. */
    FIELD_WRAPPER_DEFAULT(
      Whistle[numOfPlayers], nomadz_msgs::msg::TeamMateData::whistle, whistle), /**<Output of the WhistleRecognizer*/

    // User Defined Data - Defined

    FIELD_WRAPPER_DEFAULT(PersonalData[numOfPlayers],
                          nomadz_msgs::msg::TeamMateData::robots_personal_data,
                          robotsPersonalData), //+1 to be removed once the coach is a functioning player!

    // Initialization
    for (int i = 0; i < numOfPlayers; ++i) {
    timeStamps[i] = 0;
    timeLastGroundContact[i] = 0;
  };
});

inline unsigned char toIntegral(const TeamMateDataSelector& a) {
  return static_cast<unsigned char>(a);
}

inline TeamMateDataSelector operator|(const TeamMateDataSelector& a, const TeamMateDataSelector& b) {
  return static_cast<TeamMateDataSelector>(toIntegral(a) | toIntegral(b));
}

inline TeamMateDataSelector operator&(const TeamMateDataSelector& a, const TeamMateDataSelector& b) {
  return static_cast<TeamMateDataSelector>(toIntegral(a) & toIntegral(b));
}

/**
 * @class TeamDataSenderOutput
 * An empty dummy representation for the TeamDataSender module
 */
STREAMABLE_DECLARE_LEGACY(TeamDataSenderOutput)
STREAMABLE_LEGACY(TeamDataSenderOutput,
                  {
                    ,
                  });
