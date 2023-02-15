/**
 * @file FieldView.h
 *
 * Declaration of class FieldView
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <QString>
#include <QIcon>

#include "SimRobot.h"

class RobotConsole;

/**
 * @class FieldView
 *
 * A class to represent a view displaying debug drawings in field coordinates.
 */
class FieldView : public SimRobot::Object {
public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param name The name of the view.
   */
  FieldView(const QString& fullName, RobotConsole& console, const std::string& name);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon;       /**< The icon used for listing this view in the scene graph */
  RobotConsole& console;  /**< A reference to the console object. */
  const std::string name; /**< The name of the view. */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const { return fullName; }
  virtual const QIcon* getIcon() const { return &icon; }

  friend class FieldWidget;
};
