/**
 * @file Settings.cpp
 *
 * Implementation of a class that provides access to settings-specific configuration directories.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Settings.h"
#include "Core/Streams/InStreams.h"
#include "Core/Communication/RoboCupControlData.h"
#ifdef TARGET_ROBOT
#include "Nao/NaoBody.h"
#include <cstdio>
#endif
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"
#include "Core/Global.h"
#include "Core/Streams/StreamHandler.h"

bool Settings::recover = false;

Settings Settings::settings(true);
bool Settings::loaded = false;

Settings::Settings(bool master)
    : teamNumber(0), teamColor(blue), keeperColor(orange), playerNumber(0), location("Default"), teamPort(0) {
  ASSERT(master);
}

void Settings::init() {
  ASSERT(TEAM_BLUE == blue && TEAM_RED == red);
  if (!loaded) {
    VERIFY(settings.load());
    loaded = true;
  }
  *this = settings;
}

bool Settings::load() {
#ifdef TARGET_ROBOT
  robot = NaoBody().getName();
#else
  robot = "Nao";
#endif

#ifdef TARGET_SIM
  simulatedRobot = "Nao";
#endif

  if (!Global::theStreamHandler) {
    static StreamHandler streamHandler;
    Global::theStreamHandler = &streamHandler;
  }

  InMapFile stream("settings.cfg");
  if (stream.exists()) {
    stream >> *this;
  } else {
    TRACE("Could not load settings for robot \"%s\" from settings.cfg", robot.c_str());
    return false;
  }

#ifdef TARGET_ROBOT
  printf("teamNumber %d\n", teamNumber);
  printf("teamPort %d\n", teamPort);
  printf("teamColor %s\n", teamColor == TEAM_BLUE ? "blue" : "red");
  printf("playerNumber %d\n", playerNumber);
  printf("location %s\n", location.c_str());
#endif
  return true;
}
