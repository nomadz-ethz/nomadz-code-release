/**
 * @file Settings.h
 *
 * Definition of a class that provides access to settings-specific configuration directories.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Enum.h"
#include "Core/Streams/AutoStreamable.h"

// Nasty hack to allow RoboCupCtrl to modify the globally accessible instance
// of settings when creating new instances of ControllerRobot
#ifdef TARGET_SIM
class RoboCupCtrl;
#endif

/**
 * @class Settings
 * The class provides access to settings-specific configuration directories.
 */
STREAMABLE(Settings, {
private:
  static Settings settings; /**< The master settings instance. */
  static bool loaded;       /**< The load() of the master settings instance was called or not. */
#ifdef TARGET_SIM
  friend class RoboCupCtrl;
#endif

  /**
   * Constructor for the master settings instance.
   */
  explicit Settings(bool master);

  /**
   * Initializes the instance.
   */
  void init();

  /**
   * The function loads the settings from disk.
   * @return Whether the settings were loaded successfully.
   */
  bool load();

public:
  ENUM(TeamColor, blue, red);

  std::string simulatedRobot;
  std::string robot;   /**< The name of this robot. */
  static bool recover; /**< Start directly without the pre-initial state. */

  friend class Framework, /**< To access loaded. */

    (int)(0)teamNumber,         /**< The number of our team in the game controller. Use theOwnTeamInfo.teamNumber instead. */
    (TeamColor)(blue)teamColor, /**< The color of our team. Use theOwnTeamInfo.teamColour instead. */
    (int)(0)playerNumber,       /**< The number of the robot in the team. Use theRobotInfo.playerNumber instead. */
    (std::string)("Default")location, /**< The name of the location. */
    (int)(0)teamPort,                 /**< The UDP port our team uses for team communication. */

    // Initialization
    init();
});
