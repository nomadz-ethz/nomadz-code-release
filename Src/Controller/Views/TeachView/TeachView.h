/**
 * @file TeachView.h
 *
 * Declaration of class TeachView
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <string>
#include <QIcon>
#include <QObject>

#include "SimRobot.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/Representations/Lesson.h"
#include "Controller/Views/TeachView/TeachSession.h"

class TeachWidget;

/**
 * @class TeachView
 *
 * A class to represent a SimRobot::Object that holds all the data internal to the Teach tool.
 * Inherits QObject for signal/slot functionality.
 */
class TeachView : public QObject, public SimRobot::Object {
  Q_OBJECT

public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   */
  TeachView(const QString& fullName, ConsoleRoboCupCtrl& console);

  // Handle a console command (the part that comes after "teach ")
  void handleConsole(const std::string&);

  QMenu* createFileMenu() const;

private slots:
  void newLesson();
  void loadLesson();
  void mergeLesson();
  void saveLessonAs();

private:
  const QString fullName;      /**< The path to this view in the scene graph */
  const QIcon icon;            /**< The icon used for listing this view in the scene graph */
  ConsoleRoboCupCtrl& console; /**< A reference to the console object. */

  // Used by all of the other teach-related widgets
  Lesson lesson;
  TeachSession session; // must come AFTER lesson (constructor needs to first have an initialized lesson)

  QWidget dialogOwner;

  void mergeLessonAt(const QString& lessonPath);

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const { return fullName; }
  virtual const QIcon* getIcon() const { return &icon; }

  // These need to access data stored in here
  friend class ClassifierWidget;
  friend class DatasetWidget;
  friend class GroupWidget;
  friend class SampleWidget;
  friend class TeachWidget;
  friend class TrainWidget;
};
