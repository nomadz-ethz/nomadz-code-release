/**
 * @file OpenGLMethods.h
 *
 * Declaration of class OpenGLMethods.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/ColorReference.h"

/**
 * @class OpenGLMethods
 *
 * Defines static methods to initialize and paint with openGL
 */
class OpenGLMethods {
private:
  /** returns (value+offset)/scale */
  static float transformCoordinates(int scale, int offset, int value) { return (value + offset) / (float)scale; }

  /**
   * The method returns prototypical color values for a color class for visualization.
   * @param colorClass The color class.
   * @param r The red channel of the prototypical color.
   * @param g The green channel of the prototypical color.
   * @param b The blue channel of the prototypical color.
   */
  static void getColorClassColor(ColorClasses::Color colorClass, unsigned char& r, unsigned char& g, unsigned char& b);

public:
  /** writes a cube to an openGL list*/
  static void paintCubeToOpenGLList(int xSize,
                                    int ySize,
                                    int zSize,
                                    int listID,
                                    bool paintFullCube,
                                    int scale,
                                    int offsetX,
                                    int offsetY,
                                    int offsetZ,
                                    int r,
                                    int g,
                                    int b) {
    paintCubeToOpenGLList(0, 0, 0, xSize, ySize, zSize, listID, paintFullCube, scale, offsetX, offsetY, offsetZ, r, g, b);
  }

  /** writes a cube to an openGL list*/
  static void paintCubeToOpenGLList(int xLow,
                                    int yLow,
                                    int zLow,
                                    int xHigh,
                                    int yHigh,
                                    int zHigh,
                                    int listID,
                                    bool paintFullCube,
                                    int scale,
                                    int offsetX,
                                    int offsetY,
                                    int offsetZ,
                                    int r,
                                    int g,
                                    int b,
                                    bool beginList = true,
                                    bool endList = true);

  /** writes the pixels of the image to an openGL list*/
  static void paintImagePixelsToOpenGLList(const Image& image, int colorModel, int zComponent, bool polygon, int listID) {
    paintImagePixelsToOpenGLList(image, colorModel, zComponent, polygon, listID, 0, image.width, 0, image.height);
  }

  /** writes the pixels of the image between y1 and y2 to an openGL list*/
  static void paintImagePixelsToOpenGLList(
    const Image& image, int colorModel, int zComponent, bool polygon, int listID, int x1, int x2, int y1, int y2);

  /** writes one color of the color reference to an openGL list */
  static void paintColorReference(const ColorReference& colorReference, ColorClasses::Color color, int listID);
};
