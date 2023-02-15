/**
 * @file CABSLGraphView.h
 *
 * Declaration of a class to represent a view displaying the cabsl option call graph of a behavior.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "DotView.h"

class RobotConsole;

/**
 * A class to represent a view displaying the cabsl option call graph of a behavior.
 */
class CABSLGraphViewObject : public DotViewObject {
public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param behaviorName The name of the behavior directory in the BehaviorControl folder
   * @param optionsFile The name of the file that includes all other option files
   */
  CABSLGraphViewObject(const QString& fullName, const QString& behaviorName, const QString& optionsFile);

private:
  class Option {
    QString originalLine, filePath, optionName, dotColorDesc;
    QStringList calls;

  public:
    Option(const QString& line);
    void findCalls(const QString& behaviorName, QList<Option*>& options);
    QString declareNode();
    QString drawCalls();
    QString pattern();
  };

  QString behaviorName; /**< The name of the behavior directory in the BehaviorControl folder */
  QString optionsFile;  /**< The name of the file that includes all other option files */

  /**
   * Checks whether the content that will be returned from a \c generateDotFileContent call has changed
   * @return \c true When \c generateDotFileContent will return something new
   */
  virtual bool hasChanged() { return false; }

  /**
   * Returns the content of the dot graph file that will be displayed
   * @return The content of the dot graph file
   */
  virtual QString generateDotFileContent();
};
