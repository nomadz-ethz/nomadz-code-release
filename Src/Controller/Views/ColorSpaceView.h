/**
 * @file ColorSpaceView.h
 *
 * Declaration of class ColorSpaceView
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "View3D.h"
#include "Core/Enum.h"

class RobotConsole;

/**
 * @class ColorSpaceView
 *
 * A class to represent a view with information about the timing of modules.
 */

class ColorSpaceView : public View3D {
public:
  /**
   * The image formats available.
   */
  ENUM(ColorModel, YCbCr, RGB, HSI, user);

  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param c The console object.
   * @param n The name of the image to display.
   * @param cm The color model in which the image should be displayed by this view.
   * @param ch The channel to display (1..3) or 0 to display all channels.
   * @param b The background color.
   */
  ColorSpaceView(const QString& fullName,
                 RobotConsole& c,
                 const std::string& n,
                 ColorModel cm,
                 int ch,
                 const Vector3<>& b,
                 bool upperCam);

  /**
   * The function returns the name of a channel in a certain color model as string.
   * @param cm The color model.
   * @param channel The channel in the color model.
   * @return The string representation of the name of the channel in the color model.
   */
  static const char* getChannelNameForColorModel(ColorModel cm, int channel) {
    static const char* names[][4] = {
      {"all", "Cb", "Y", "Cr"}, {"all", "R", "G", "B"}, {"all", "H", "S", "I"}, {"all", "0", "1", "2"}};
    return names[cm][channel];
  }

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

  /**
   * The function returns the view distance.
   * @return The distance from which the scene is viewed.
   */
  virtual float getViewDistance() const { return channel ? 5.0f : 8.0f; }

private:
  RobotConsole& console;  /**< A reference to the console object. */
  std::string name;       /**< The name of the image. */
  ColorModel colorModel;  /**< The color model in which the image should be displayed by this view. */
  int channel;            /**< The channel to display (1..3) or 0 to display all channels. */
  unsigned lastTimeStamp; /**< The frame number of last image that was drawn. */
  bool upperCam;
};
