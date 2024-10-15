#pragma once

#include <cstdint>

#include "nomadz_communication_msgs/robocup_game_control_data_enums.hpp"

#define GAMECONTROLLER_STRUCT_HEADER "RGme"
#define GAMECONTROLLER_STRUCT_VERSION 18

namespace nomadz_communication {
  constexpr int MAX_NUM_PLAYERS = 20;

  struct RobotInfo {
    nomadz_communication_msgs::Penalty penalty; // penalty state of the player
    uint8_t secs_till_unpenalised;              // estimate of time till unpenalised
  };

  struct TeamInfo {
    uint8_t team_id;                                                // unique team number
    nomadz_communication_msgs::TeamJerseyColor field_player_colour; // colour of the field players
    nomadz_communication_msgs::TeamJerseyColor goal_keeper_colour;  // colour of the goalkeeper
    uint8_t goal_keeper;                                            // player number of the goalkeeper (1-MAX_NUM_PLAYERS)
    uint8_t score;                                                  // team's score
    uint8_t penalty_shot;                                           // penalty shot counter
    uint16_t single_shots;                                          // bits represent penalty shot success
    uint16_t message_budget; // number of team messages the team is allowed to send for the remainder of the game
    struct RobotInfo players[MAX_NUM_PLAYERS]; // the team's players
  };

  struct RobocupGameControlData {
    char header[4];           // header to identify the structure
    uint8_t version;          // version of the data structure
    uint8_t packet_number;    // number incremented with each packet sent (with wraparound)
    uint8_t players_per_team; // the number of players on a team
    nomadz_communication_msgs::CompetitionPhase
      competition_phase; // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
    nomadz_communication_msgs::CompetitionType
      competition_type; // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_DYNAMIC_BALL_HANDLING)
    nomadz_communication_msgs::GamePhase game_phase; // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
    nomadz_communication_msgs::GameState state;      // state of the game (STATE_READY, STATE_PLAYING, etc)
    nomadz_communication_msgs::SetPlay set_play;     // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_KICK, etc)
    uint8_t first_half;                              // 1 = game in first half, 0 otherwise
    uint8_t kicking_team;                            // the team number of the next team to kick off, free kick etc
    int16_t secs_remaining;                          // estimate of number of seconds remaining in the half
    int16_t secondary_time; // number of seconds shown as secondary time (remaining ready, until free ball, etc)
    struct TeamInfo teams[2];
  };
} // namespace nomadz_communication
