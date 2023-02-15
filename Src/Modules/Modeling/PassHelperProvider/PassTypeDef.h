/**
 * @file PassTypeDef.h
 *
 * PassTypeDef declares all the typedef used by the pass algorithms
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once
// Definition of different type, just to make the code more readable
// Note: if you put the Vector2<int> you cannot use anymore some methods implemented in PassHelperProvider, because you
// cannot perform some operation implemented in
// Vector2 class

typedef Vector2<float> Location;
typedef Vector2<float> Point2D;
typedef std::pair<Location, bool>
  LocTarget; // This pair contain the location of a point and if it's valid for a pass (true) or not (false)
typedef std::pair<int, int> Index; // index for (row, column), referred to all the candidate points
