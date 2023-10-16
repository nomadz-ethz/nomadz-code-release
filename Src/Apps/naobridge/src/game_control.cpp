/**
 * @file game_ctrl.cpp
 *
 * Implementation of a NAOqi library that communicates with the GameController.
 * It provides the data received in ALMemory.
 * It also implements the official button interface and sets the LEDs as
 * specified in the rules.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#include "game_control.h"

#include "Core/System/UdpComm.h"
#include "SPL/VisualRefereeChallenge.h"

#include <cstring>
#include <memory>

#include "lola_bridge.h"

static const int BUTTON_DELAY = 30; /**< Button state changes are ignored when happening in less than 30 ms. */
static const int GAMECONTROLLER_TIMEOUT =
  2000;                              /**< Connected to GameController when packet was received within the last 2000 ms. */
static const int ALIVE_DELAY = 1000; /**< Send an alive signal every 1000 ms. */

// internal logic
std::shared_ptr<UdpComm> udp;             /**< The socket used to communicate. */
in_addr gameControllerAddress;            /**< The address of the GameController PC. */
int teamNum;                              /**< The team number. */
bool previousChestButtonPressed;          /**< Whether the chest button was pressed during the previous cycle. */
bool previousLeftFootButtonPressed;       /**< Whether the left foot bumper was pressed during the previous cycle. */
bool previousRightFootButtonPressed;      /**< Whether the right foot bumper was pressed during the previous cycle. */
unsigned whenChestButtonStateChanged;     /**< When last state change of the chest button occured (DCM time). */
unsigned whenLeftFootButtonStateChanged;  /**< When last state change of the left foot bumper occured (DCM time). */
unsigned whenRightFootButtonStateChanged; /**< When last state change of the right foot bumper occured (DCM time). */
unsigned whenPacketWasReceived;           /**< When the last GameController packet was received (DCM time). */
unsigned whenPacketWasSent;               /**< When the last return packet was sent to the GameController (DCM time). */

/**
 * Resets the internal for the control logic
 */
void ctrl_init() {
  memset(&gameControllerAddress, 0, sizeof(gameControllerAddress));
  teamNum = (uint8_t)0;
  previousChestButtonPressed = false;
  previousLeftFootButtonPressed = false;
  previousRightFootButtonPressed = false;
  whenChestButtonStateChanged = 0;
  whenLeftFootButtonStateChanged = 0;
  whenRightFootButtonStateChanged = 0;
  whenPacketWasReceived = 0;
  whenPacketWasSent = 0;

  if (!udp) {
    udp = std::make_shared<UdpComm>();
    if (!udp->setBlocking(false) || !udp->setBroadcast(false) || !udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) ||
        !udp->setLoopback(false)) {
      fprintf(stderr, "naobridge: Could not open UDP port\n");
      udp.reset();
      udp = nullptr;
      // continue, because button interface will still work
    }
  }

  if (shared_data->robotInfo[fieldPlayerColour] == 0) {
    shared_data->robotInfo[fieldPlayerColour] = TEAM_BLACK;
  }
}

/**
 * Sets the LEDs whenever the state they visualize changes.
 * Regularily sends the return packet to the GameController.
 */
void ctrl_handle_output(float* actuators, const RoboCup::RoboCupGameControlData& gameCtrlData) {
  unsigned now = curTime;
  int playerNum = shared_data->robotInfo[playerNumber];

  if (teamNum && playerNum && playerNum <= gameCtrlData.playersPerTeam &&
      (gameCtrlData.teams[0].teamNumber == teamNum || gameCtrlData.teams[1].teamNumber == teamNum)) {
    const RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == teamNum ? 0 : 1];
    switch (team.teamColor) {
    case TEAM_BLUE:
      ctrl_set_led(actuators, lFootLedRedActuator, 0.f, 0.f, 1.f);
      break;
    case TEAM_RED:
      ctrl_set_led(actuators, lFootLedRedActuator, 1.f, 0.f, 0.f);
      break;
    case TEAM_YELLOW:
      ctrl_set_led(actuators, lFootLedRedActuator, 1.f, 1.f, 0.f);
      break;
    case TEAM_WHITE:
      ctrl_set_led(actuators, lFootLedRedActuator, 1.f, 1.f, 1.f);
      break;
    case TEAM_GREEN:
      ctrl_set_led(actuators, lFootLedRedActuator, 0.f, 1.f, 0.f);
      break;
    case TEAM_ORANGE:
      ctrl_set_led(actuators, lFootLedRedActuator, 1.f, 0.5f, 0.f);
      break;
    case TEAM_PURPLE:
      ctrl_set_led(actuators, lFootLedRedActuator, 1.f, 0.f, 1.f);
      break;
    case TEAM_BROWN:
      ctrl_set_led(actuators, lFootLedRedActuator, 0.2f, 0.1f, 0.f);
      break;
    case TEAM_GRAY:
      ctrl_set_led(actuators, lFootLedRedActuator, 0.2f, 0.2f, 0.2f);
      break;
    case TEAM_BLACK:
    default:
      ctrl_set_led(actuators, lFootLedRedActuator, 0.f, 0.f, 0.f);
    }

    if (gameCtrlData.state == STATE_INITIAL && gameCtrlData.gamePhase == GAME_PHASE_PENALTYSHOOT &&
        gameCtrlData.kickingTeam == team.teamNumber)
      ctrl_set_led(actuators, rFootLedRedActuator, 0.f, 1.f, 0.f);
    else if (gameCtrlData.state == STATE_INITIAL && gameCtrlData.gamePhase == GAME_PHASE_PENALTYSHOOT &&
             gameCtrlData.kickingTeam != team.teamNumber)
      ctrl_set_led(actuators, rFootLedRedActuator, 1.f, 1.0f, 0.f);
    else if (now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT && gameCtrlData.state <= STATE_SET &&
             gameCtrlData.kickingTeam == team.teamNumber)
      ctrl_set_led(actuators, rFootLedRedActuator, 1.f, 1.f, 1.f);
    else
      ctrl_set_led(actuators, rFootLedRedActuator, 0.f, 0.f, 0.f);

    if (team.players[playerNum - 1].penalty != PENALTY_NONE)
      ctrl_set_led(actuators, chestBoardLedRedActuator, 1.f, 0.f, 0.f);
    else
      switch (gameCtrlData.state) {
      case STATE_READY:
        ctrl_set_led(actuators, chestBoardLedRedActuator, 0.f, 0.f, 1.f);
        break;
      case STATE_SET:
        ctrl_set_led(actuators, chestBoardLedRedActuator, 1.f, 0.4f, 0.f);
        break;
      case STATE_PLAYING:
        ctrl_set_led(actuators, chestBoardLedRedActuator, 0.f, 1.f, 0.f);
        break;
      default:
        ctrl_set_led(actuators, chestBoardLedRedActuator, 0.f, 0.f, 0.f);
      }

    if (now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT && now - whenPacketWasSent >= ALIVE_DELAY && ctrl_send())
      whenPacketWasSent = now;
  }
}

/**
 * Sets states in the LED request.
 * @param led The index of the red channel of an RGB LED.
 * @param red The red intensity [0..1].
 * @param green The green intensity [0..1].
 * @param blue The blue intensity [0..1].
 */
void ctrl_set_led(float* actuators, LBHActuatorIds led, float red, float green, float blue) {
  actuators[led] = red;
  actuators[led + 1] = green;
  actuators[led + 2] = blue;
}

/**
 * Handles the button interface.
 * Resets the internal state when a new team number was set.
 * Receives packets from the GameController.
 * Initializes gameCtrlData when teamNumber and playerNumber are available.
 */
void ctrl_handle_input(const float* sensors,
                       RoboCup::RoboCupGameControlData& gameCtrlData,
                       bool active,
                       bool resetToInitial) {
  unsigned now = curTime;
  int playerNum = shared_data->robotInfo[playerNumber];

  if (shared_data->robotInfo[teamNumber] != teamNum) {
    // new team number was set -> reset internal structure
    ctrl_init();
    teamNum = shared_data->robotInfo[teamNumber];
    memset(&gameCtrlData, 0, sizeof(gameCtrlData));
  }

  if (ctrl_receive(gameCtrlData)) {
    whenPacketWasReceived = now;
  }

  if (teamNum && playerNum) {
    // init gameCtrlData if invalid
    if (gameCtrlData.teams[0].teamNumber != teamNum && gameCtrlData.teams[1].teamNumber != teamNum) {
      uint8_t teamColor = (uint8_t)shared_data->robotInfo[fieldPlayerColour];
      if (teamColor != TEAM_BLUE && teamColor != TEAM_RED && teamColor != TEAM_YELLOW && teamColor != TEAM_WHITE &&
          teamColor != TEAM_GREEN && teamColor != TEAM_ORANGE && teamColor != TEAM_PURPLE && teamColor != TEAM_BROWN &&
          teamColor != TEAM_GRAY)
        teamColor = TEAM_BLACK;
      gameCtrlData.teams[0].teamNumber = (uint8_t)teamNum;
      gameCtrlData.teams[0].teamColor = teamColor;
      gameCtrlData.teams[1].teamColor = teamColor ^ 1; // we don't know better
      if (!gameCtrlData.playersPerTeam)
        gameCtrlData.playersPerTeam = (uint8_t)playerNum; // we don't know better
    }
    RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == teamNum ? 0 : 1];

    if (playerNum <= gameCtrlData.playersPerTeam) {
      // if active in the main state machine allow configuration
      if (active) {
        bool chestButtonPressed = sensors[chestButtonSensor] != 0.f;
        if (chestButtonPressed != previousChestButtonPressed && now - whenChestButtonStateChanged >= BUTTON_DELAY) {
          if (chestButtonPressed && gameCtrlData.state == STATE_INITIAL) {
            // in initial state always go to penalized
            RoboCup::RobotInfo& player = team.players[playerNum - 1];
            player.penalty = PENALTY_MANUAL;
            gameCtrlData.state = STATE_PLAYING;
          } else if (chestButtonPressed && (now - whenPacketWasReceived >= GAMECONTROLLER_TIMEOUT)) {
            // ignore the chest buttons in all other states when game controller is active
            RoboCup::RobotInfo& player = team.players[playerNum - 1];
            if (player.penalty == PENALTY_NONE) {
              player.penalty = PENALTY_MANUAL;
            } else {
              player.penalty = PENALTY_NONE;
              gameCtrlData.state = STATE_PLAYING;
            }
          }
          previousChestButtonPressed = chestButtonPressed;
          whenChestButtonStateChanged = now;
        }
      } else if (gameCtrlData.state == STATE_PLAYING && now - whenPacketWasReceived >= GAMECONTROLLER_TIMEOUT) {
        // Emulate penalized if not active but in playing state without GameController (saves chest button press).
        RoboCup::RobotInfo& player = team.players[playerNum - 1];
        player.penalty = PENALTY_MANUAL;
      }

      if (resetToInitial) {
        RoboCup::RobotInfo& player = team.players[playerNum - 1];
        gameCtrlData.state = STATE_INITIAL;
        player.penalty = PENALTY_NONE;
      }
    } else {
      fprintf(stderr, "Player number %d too big. Maximum number is %d.\n", playerNum, gameCtrlData.playersPerTeam);
    }
  }
}

/**
 * Sends the return packet to the GameController.
 */
bool ctrl_send() {
  RoboCup::RoboCupGameControlReturnData returnPacket;
  // Visual Referee Challenge message
  int referee_signal = shared_data->robotInfo[refereeChallengeSignal];
  if (referee_signal > 0) {
    returnPacket.version = GAMECONTROLLER_RETURN_STRUCT_VRC_VERSION;
    returnPacket.fallen = (uint8_t)referee_signal;
    returnPacket.ballAge = (float)shared_data->robotInfo[refereeChallengeTimeSinceWhistle];
  } else {
    returnPacket.fallen = (uint8_t)shared_data->robotInfo[playerFallen];
    returnPacket.ballAge = (float)(shared_data->robotInfo[playerBallAge] / 1000.0);
  }
  returnPacket.playerNum = (uint8_t)shared_data->robotInfo[playerNumber];
  returnPacket.teamNum = (uint8_t)shared_data->robotInfo[teamNumber];
  returnPacket.pose[0] = (float)(shared_data->robotInfo[playerPoseX] / 1000.0);
  returnPacket.pose[1] = (float)(shared_data->robotInfo[playerPoseY] / 1000.0);
  returnPacket.pose[2] = (float)(shared_data->robotInfo[playerPoseTheta] / 1000.0);
  returnPacket.ball[0] = (float)(shared_data->robotInfo[playerBallX] / 1000.0);
  returnPacket.ball[1] = (float)(shared_data->robotInfo[playerBallY] / 1000.0);

  return !udp || udp->write((const char*)&returnPacket, sizeof(returnPacket) + 8);
}

/**
 * Receives a packet from the GameController.
 * Packets are only accepted when the team number is know (nonzero) and
 * they are addressed to this team.
 */
bool ctrl_receive(RoboCup::RoboCupGameControlData& gameCtrlData) {
  bool received = false;
  int size;
  RoboCup::RoboCupGameControlData buffer;
  struct sockaddr_in from;
  while (udp && (size = udp->read((char*)&buffer, sizeof(buffer), from)) > 0) {
    if (size == sizeof(buffer) && !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
        buffer.version == GAMECONTROLLER_STRUCT_VERSION && teamNum &&
        (buffer.teams[0].teamNumber == teamNum || buffer.teams[1].teamNumber == teamNum)) {
      gameCtrlData = buffer;
      if (memcmp(&gameControllerAddress, &from.sin_addr, sizeof(in_addr))) {
        memcpy(&gameControllerAddress, &from.sin_addr, sizeof(in_addr));
        udp->setTarget(inet_ntoa(gameControllerAddress), GAMECONTROLLER_RETURN_PORT);
      }

      received = true;
    }
  }
  return received;
}

/**
 * Close all resources acquired.
 * Called when initialization failed or during destruction.
 */
void ctrl_close() {
  if (udp)
    udp.reset();
}
