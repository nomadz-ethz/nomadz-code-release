/**
 * @file VisualRefereeReact.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <iostream>
#include <vector>

option(VisualRefereeReact, const int& signal, bool play_sound, bool mirror_action) {

  initial_state(initial) {
    transition {
      if (mirror_action) {
        switch (signal) {
          case RefereePercept::kickInRedTeam:
            SpecialAction(SpecialActionRequest::Ref_KickInRight);
            break;
          case RefereePercept::kickInBlueTeam:
            SpecialAction(SpecialActionRequest::Ref_KickInLeft);
            break;
          case RefereePercept::goalKickRedTeam:
            SpecialAction(SpecialActionRequest::Ref_GoalKickRight);
            break;
          case RefereePercept::goalKickBlueTeam:
            SpecialAction(SpecialActionRequest::Ref_GoalKickLeft);
            break;
          case RefereePercept::cornerKickRedTeam:
            SpecialAction(SpecialActionRequest::Ref_CornerKickRight);
            break;
          case RefereePercept::cornerKickBlueTeam:
            SpecialAction(SpecialActionRequest::Ref_CornerKickLeft);
            break;
          case RefereePercept::goalRedTeam:
            SpecialAction(SpecialActionRequest::Ref_GoalRight);
            break;
          case RefereePercept::goalBlueTeam:
            SpecialAction(SpecialActionRequest::Ref_GoalLeft);
            break;
          case RefereePercept::pushingFreeKickRedTeam:
            SpecialAction(SpecialActionRequest::Ref_PushingFreeKickRight);
            break;
          case RefereePercept::pushingFreeKickBlueTeam:
            SpecialAction(SpecialActionRequest::Ref_PushingFreeKickLeft);
            break;
          case RefereePercept::fullTime:
            SpecialAction(SpecialActionRequest::Ref_FullTime);
            break;
          case RefereePercept::playerExchangeRedTeam:
            SpecialAction(SpecialActionRequest::Ref_PlayerExchangeRight);
            break;
          case RefereePercept::playerExchangeBlueTeam:
            SpecialAction(SpecialActionRequest::Ref_PlayerExchangeLeft);
            break;
        }
      }
      if (play_sound) {
        switch (signal) {
          case RefereePercept::kickInRedTeam:
            PlaySound("kickIn.wav");
            PlaySound("redTeam.wav");
            break;
          case RefereePercept::kickInBlueTeam:
            PlaySound("kickIn.wav");
            PlaySound("blueTeam.wav");
            break;
          case RefereePercept::goalKickRedTeam:
            PlaySound("goalKick.wav");
            PlaySound("redTeam.wav");
            break;
          case RefereePercept::goalKickBlueTeam:
            PlaySound("goalKick.wav");
            PlaySound("blueTeam.wav");
            break;
          case RefereePercept::cornerKickRedTeam:
            PlaySound("cornerKick.wav");
            PlaySound("redTeam.wav");
            break;
          case RefereePercept::cornerKickBlueTeam:
            PlaySound("cornerKick.wav");
            PlaySound("blueTeam.wav");
            break;
          case RefereePercept::goalRedTeam:
            PlaySound("goal.wav");
            PlaySound("redTeam.wav");
            break;
          case RefereePercept::goalBlueTeam:
            PlaySound("goal.wav");
            PlaySound("blueTeam.wav");
            break;
          case RefereePercept::pushingFreeKickRedTeam:
            PlaySound("pushingFreekick.wav");
            PlaySound("redTeam.wav");
            break;
          case RefereePercept::pushingFreeKickBlueTeam:
            PlaySound("pushingFreekick.wav");
            PlaySound("blueTeam.wav");
            break;
          case RefereePercept::fullTime:
            PlaySound("fullTime.wav");
            break;
          case RefereePercept::playerExchangeRedTeam:
            PlaySound("redTeam.wav"); // NOTE: no sound exists for playerExchange
            PlaySound("redTeam.wav");
            break;
          case RefereePercept::playerExchangeBlueTeam:
            PlaySound("blueTeam.wav"); // NOTE: no sound exists for playerExchange
            PlaySound("blueTeam.wav");
            break;
        }
      }
      goto endRefereeReact;
    }
  }

  target_state(endRefereeReact) {}
}
