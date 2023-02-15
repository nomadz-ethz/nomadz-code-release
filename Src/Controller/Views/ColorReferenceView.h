/**
 * @file ColorReferenceView.h
 *
 * The file declares a class to visualize the color reference.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "View3D.h"
#include "Core/ColorClasses.h"

class RobotConsole;

/**
 * @class ColorReferenceView
 * A class to visualize the color reference.
 */
class ColorReferenceView : public View3D {
public:
  /**
   * Constructor.
   * @param fullName The full name of the view.
   * @param c The console object.
   * @param color The color that is visualized.
   * @param b The background color.
   */
  ColorReferenceView(const QString& fullName, RobotConsole& c, ColorClasses::Color color, const Vector3<>& b);

protected:
  /**
   * Update the display lists if required.
   */
  virtual void updateDisplayLists();

  /**
   * Need the display lists to be updated?
   * @return Yes or no?
   */
  virtual bool needsUpdate() const;

private:
  RobotConsole& console;     /**< A reference to the console object. */
  ColorClasses::Color color; /**< The color that is visualized. */
  unsigned lastTimeStamp;    /**< The time stamp of last color table that was drawn. */
};
