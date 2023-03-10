/**
 * @file ModuleGraphView.h
 *
 * Declaration of a class to represent a view displaying the module layout of the process.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a> and Colin Graf
 */

#pragma once

#include "DotView.h"

#include <string>

class RobotConsole;

/**
 * A class to represent a view displaying the module layout of the process.
 */
class ModuleGraphViewObject : public DotViewObject {
public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param processIdentifier The identifier of the process the modules of which are displayed.
   * @param category The category of the modules of this view. If "", show all categories.
   */
  ModuleGraphViewObject(const QString& fullName, RobotConsole& console, char processIdentifier, const std::string& category);

private:
  RobotConsole& console;               /**< A reference to the console object. */
  char processIdentifier;              /**< The name of the view. */
  std::string category;                /**< The category of the modules of this view. If "", show all categories. */
  unsigned int lastModulInfoTimeStamp; /**< Module Info timestamp when the image was created. */

  /**
   * The method replaces all ' ' by '_'.
   * @param s The input string.
   * @return The string in which spaces were replaced.
   */
  std::string compress(const std::string& s) const;

  /**
   * Checks whether the content that will be returned from a \c generateDotFileContent call has changed
   * @return \c true When \c generateDotFileContent will return something new
   */
  virtual bool hasChanged();

  /**
   * Returns the content of the dot graph file that will be displayed
   * @return The content of the dot graph file
   */
  virtual QString generateDotFileContent();
};
