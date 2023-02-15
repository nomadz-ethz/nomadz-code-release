/**
 * @file PlayerPercept.cpp
 *
 * Representation of players detected in the image
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "PlayerPercept.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"

void PlayerPercept::draw() const {
  DECLARE_DEBUG_DRAWING("representation:PlayerPercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:PlayerPercept:Field", "drawingOnField");

  COMPLEX_DRAWING("representation:PlayerPercept:Image", {
    for (const auto& player : players) {
      const ColorRGBA color =
        player.detectedJersey ? (player.opponent ? ColorClasses::red : ColorClasses::blue) : ColorClasses::black;

      LINE("representation:PlayerPercept:Image", player.x1, player.y2, player.x2, player.y2, 3, Drawings::ps_solid, color);

      if (!player.standing) {
        CROSS("representation:PlayerPercept:Image",
              0.5f * (player.x1 + player.x2),
              player.y2,
              2,
              2,
              Drawings::ps_solid,
              ColorRGBA(128, 128, 128, 255));

      } else {
        CROSS(
          "representation:PlayerPercept:Image", 0.5f * (player.x1 + player.x2), player.y2, 2, 2, Drawings::ps_solid, color);

        RECTANGLE("representation:PlayerPercept:Image",
                  player.x1,
                  player.jerseyY0,
                  player.x2,
                  player.jerseyY1,
                  1,
                  Drawings::ps_solid,
                  color);
      }
    }
  });

  COMPLEX_DRAWING("representation:PlayerPercept:Field", {
    for (const auto& player : players) {
      const ColorRGBA color =
        player.detectedJersey ? (player.opponent ? ColorClasses::red : ColorClasses::blue) : ColorClasses::black;

      CIRCLE("representation:PlayerPercept:Field",
             player.centerOnField.x,
             player.centerOnField.y,
             player.radius / 2.f,
             20,
             Drawings::ps_solid,
             color,
             Drawings::bs_null,
             ColorClasses::none);

      if (!player.standing) {
        CROSS("representation:PlayerPercept:Field",
              player.centerOnField.x,
              player.centerOnField.y,
              50,
              20,
              Drawings::ps_solid,
              ColorRGBA(128, 128, 128, 255));
      } else {
        CROSS("representation:PlayerPercept:Field",
              player.centerOnField.x,
              player.centerOnField.y,
              30,
              12,
              Drawings::ps_solid,
              color);
      }
    }
  });
}
