std_msgs/Header header          # Note: Not to confuse with the robocup gamecontrol (RCGCD) header which is useless. We include the timestamp of the message here and ommited the RCGCD header and version
uint8 packet_number
uint8 players_per_team

uint8 competition_phase         # robocup_game_control_data_enum, CompetitionPhase
uint8 competition_type          # robocup_game_control_data_enum, CompetitionType
uint8 game_phase                # robocup_game_control_data_enum, GamePhase
uint8 state                     # robocup_game_control_data_enum, State
uint8 set_play                  # robocup_game_control_data_enum, SetPlay

uint8 first_half
uint8 kicking_team
int16 secs_remaining
int16 secondary_time
nomadz_communication_msgs/TeamInfo[2] teams
