/**
 * @file PassTypes.h
 *
 * Describes some struct and class used by the PassHelperProvider
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Enum.h"
#include "Core/Debugging/DebugDrawings.h"
#include "PassTypeDef.h"

// Struct containing the default style used for every plot on the worldstate
struct Color {
  Color() // Initialize with default value
  {
    penWidth = 300;
    penStyle = Drawings::ps_solid;
    penColor = ColorClasses::black;
    fillColor = ColorClasses::none;
    fillStyle = Drawings::bs_null;
  }

  int penWidth;
  Drawings::PenStyle penStyle;
  ColorClasses::Color penColor;
  ColorClasses::Color fillColor;
  Drawings::FillStyle fillStyle;
};

// Struct containing all the information needed for the drawing of the shadows of the robots
struct drawingShadowPoints {
  Point2D cirlceCenter;
  Point2D pointUp;
  Point2D pointDown;
};

// Template class that given two points return us the straight line parameters connecting the two
template <class V = float> class StraightLine {
public:
  float m;
  float q;

  StraightLine();

  StraightLine(Vector2<V> point1, Vector2<V> point2) {
    Vector2<V> direction = point1 - point2;

    // y = m*x + q  --> q = y - m*x
    m = direction.y / direction.x;
    q = point1.y - m * point1.x;
  }
};

// Enumeration that define the type of pass (N = Nothing)
ENUM(TypePass, forward, lateral, backward, shoot, N);
