#pragma once

#include <cstdint>

#define GAMECONTROLLER_RETURN_STRUCT_HEADER "RGrt"
#define GAMECONTROLLER_RETURN_STRUCT_VERSION 4

namespace nomadz_communication {
  struct RobocupGameControlReturnData {
    char header[4];     // "RGrt"
    uint8_t version;    // has to be set to GAMECONTROLLER_RETURN_STRUCT_VERSION
    uint8_t player_num; // player number starts with 1
    uint8_t team_num;   // team number
    uint8_t has_fallen; // 1 means that the robot is fallen, 0 means that the robot can play

    // position and orientation of robot
    // coordinates in millimeters
    // 0,0 is in center of field
    // +ve x-axis points towards the goal we are attempting to score on
    // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
    // angle in radians, 0 along the +x axis, increasing counter clockwise
    float pose[3]; // x,y,theta

    // ball information
    float ball_age; // seconds since this robot last saw the ball. -1.f if we haven't seen it

    // position of ball relative to the robot
    // coordinates in millimeters
    // 0,0 is in center of the robot
    // +ve x-axis points forward from the robot
    // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
    float ball[2];
  };
} // namespace nomadz_communication
