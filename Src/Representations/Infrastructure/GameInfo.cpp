/**
 * @file GameInfo.cpp
 *
 * The file implements a class that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "GameInfo.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "Core/Math/Vector3.h"
#include <cstring>

#ifndef RELEASE
static void drawDigit(int digit, const Vector3<>& pos, float size, const ColorRGBA& color) {
  static const Vector3<> points[8] = {Vector3<>(1, 0, 1),
                                      Vector3<>(1, 0, 0),
                                      Vector3<>(0, 0, 0),
                                      Vector3<>(0, 0, 1),
                                      Vector3<>(0, 0, 2),
                                      Vector3<>(1, 0, 2),
                                      Vector3<>(1, 0, 1),
                                      Vector3<>(0, 0, 1)};
  static const unsigned char digits[10] = {0x3f, 0x0c, 0x76, 0x5e, 0x4d, 0x5b, 0x7b, 0x0e, 0x7f, 0x5f};
  digit = digits[std::abs(digit)];
  for (int i = 0; i < 7; ++i) {
    if (digit & (1 << i)) {
      Vector3<> from = pos - points[i] * size;
      Vector3<> to = pos - points[i + 1] * size;
      LINE3D("representation:GameInfo", from.x, from.y, from.z, to.x, to.y, to.z, 2, color);
    }
  }
}
#endif

void GameInfo::draw() const {
  DECLARE_DEBUG_DRAWING3D("representation:GameInfo", "field", {
    int mins = std::abs((int)(short)secsRemaining) / 60;
    int secs = std::abs((int)(short)secsRemaining) % 60;
    ColorRGBA color = (short)secsRemaining < 0 ? ColorRGBA(255, 0, 0) : ColorRGBA();
    drawDigit(mins / 10, Vector3<>(-350, 3500, 1000), 200, color);
    drawDigit(mins % 10, Vector3<>(-80, 3500, 1000), 200, color);
    drawDigit(secs / 10, Vector3<>(280, 3500, 1000), 200, color);
    drawDigit(secs % 10, Vector3<>(550, 3500, 1000), 200, color);
    LINE3D("representation:GameInfo", 0, 3500, 890, 0, 3500, 910, 3, color);
    LINE3D("representation:GameInfo", 0, 3500, 690, 0, 3500, 710, 3, color);
  });

  DECLARE_DEBUG_DRAWING("representation:GameInfo", "drawingOnField", {
    DRAWTEXT("representation:GameInfo",
             -5000,
             -3200,
             200,
             ColorClasses::white,
             "Time remaining: " << (int)(secsRemaining / 60) << ":" << (secsRemaining % 60));
    DRAWTEXT("representation:GameInfo", -5000, -3400, 200, ColorClasses::white, (firstHalf ? "First" : "Second") << " half");
    std::string stateStr;
    switch (state) {
    case STATE_INITIAL:
      stateStr = "Initial";
      break;
    case STATE_READY:
      stateStr = "Ready";
      break;
    case STATE_SET:
      stateStr = "Set";
      break;
    case STATE_PLAYING:
      stateStr = "Playing";
      break;
    case STATE_FINISHED:
      stateStr = "Finished";
      break;
    default:
      stateStr = "Unknown";
    }

    DRAWTEXT("representation:GameInfo", -3500, 2200, 14, ColorClasses::white, "State: " << stateStr);
  });
}
