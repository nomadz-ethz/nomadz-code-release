/**
 * @file TeamInfo.cpp
 *
 * The file implements a class that encapsulates the structure TeamInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TeamInfo.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "Core/Math/Vector3.h"
#include "Core/Settings.h"
#include <cstring>

static void drawDigit(int digit, const Vector3<>& pos, float size, int teamColor) {
  static const Vector3<> points[8] = {Vector3<>(1, 0, 1),
                                      Vector3<>(1, 0, 0),
                                      Vector3<>(0, 0, 0),
                                      Vector3<>(0, 0, 1),
                                      Vector3<>(0, 0, 2),
                                      Vector3<>(1, 0, 2),
                                      Vector3<>(1, 0, 1),
                                      Vector3<>(0, 0, 1)};
  static const unsigned char digits[10] = {0x3f, 0x0c, 0x76, 0x5e, 0x4d, 0x5b, 0x7b, 0x0e, 0x7f, 0x5f};
  ColorRGBA color = teamColor == TEAM_BLUE ? ColorRGBA(0, 0, 255) : ColorRGBA(255, 0, 0);
  digit = digits[std::abs(digit)];
  for (int i = 0; i < 7; ++i) {
    if (digit & (1 << i)) {
      Vector3<> from = pos - points[i] * size;
      Vector3<> to = pos - points[i + 1] * size;
      LINE3D("representation:TeamInfo", from.x, from.y, from.z, to.x, to.y, to.z, 2, color);
    }
  }
}

void TeamInfo::draw() const {
  DECLARE_DEBUG_DRAWING3D("representation:TeamInfo", "field");
  {
    float x = teamColor == TEAM_BLUE ? -1535.f : 1465.f;
    drawDigit(score / 10, Vector3<>(x, 3500, 1000), 200, teamColor);
    drawDigit(score % 10, Vector3<>(x + 270, 3500, 1000), 200, teamColor);
  };
}

OwnTeamInfo::OwnTeamInfo() {
  teamColor = &Global::getSettings() ? Global::getSettings().teamColor : TEAM_BLUE;
}

void OwnTeamInfo::draw() const {
  // do base class drawing first.
  TeamInfo::draw();

  DECLARE_DEBUG_DRAWING("representation:OwnTeamInfo", "drawingOnField", {
    DRAWTEXT("representation:OwnTeamInfo", -5000, -3800, 140, ColorClasses::red, (teamColor == TEAM_BLUE ? "Blue" : "Red"));
  });
}

OpponentTeamInfo::OpponentTeamInfo() {
  teamColor = 1 - (&Global::getSettings() ? Global::getSettings().teamColor : TEAM_BLUE);
}
