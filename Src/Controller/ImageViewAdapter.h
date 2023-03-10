/**
 * @file ImageViewAdapter.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#pragma once

#include "Core/Math/Vector2.h"
#include <string>
#include <map>
#include <memory>

class PointListener {
public:
  virtual void deliverPoint(const Vector2<int>& point) = 0;
  virtual ~PointListener();
};

class ImageViewAdapter {
private:
  static std::multimap<const std::string, PointListener*> listeners;

public:
  static void fireClick(const std::string view, const Vector2<int>& point);
  static bool addListener(PointListener* listener, const std::string view);
  static void removeListener(PointListener* listener, const std::string view);
  static void removeListener(PointListener* listener);
};
