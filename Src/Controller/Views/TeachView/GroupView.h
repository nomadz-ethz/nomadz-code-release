/**
 * @file GroupView.h
 *
 * Declaration of classes GroupView & GroupWidget
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <string>
#include <unordered_map>
#include <QIcon>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include "SimRobot.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/Representations/Lesson.h"
#include "Controller/Views/TeachView/TeachSession.h"
#include "Controller/Views/TeachView/TeachView.h"

/**
 * @class GroupView
 *
 * A view that shows the currently loaded groups.
 */
class GroupView : public SimRobot::Object {
public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param info The timing info object to be visualized.
   */
  GroupView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView);

private:
  const QString fullName;      /**< The path to this view in the scene graph */
  const QIcon icon;            /**< The icon used for listing this view in the scene graph */
  ConsoleRoboCupCtrl& console; /**< A reference to the console object. */
  TeachView& teachView;

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const { return fullName; }
  virtual const QIcon* getIcon() const { return &icon; }

  friend class GroupWidget; // GroupWidget needs to access data stored in here
};

/**A widget that is used inside a GroupView to display groups*/
class GroupWidget : public QWidget, public SimRobot::Widget {
  Q_OBJECT;

public:
  GroupWidget(GroupView& groupView);

  virtual QMenu* createFileMenu() const { return teachView.createFileMenu(); }
  virtual QWidget* getWidget();

private slots:
  // GroupWidget -> Lesson
  void createGroup();
  void handleDoubleClick(int, int);
  void sendEditedGroup(QTableWidgetItem*);
  void sendSelectedGroups();
  void sendVisibleGroups();

  // Lesson -> GroupWidget
  void addGroups(const std::vector<Group*>&);
  void removeGroups(const std::set<Group*>&);
  void updateCheckboxes(const std::vector<Group*>&);
  void updateGroup(Group*);

private:
  enum TableCol { VisibilityCol, ColorCol, NameCol, SizeCol };

  GroupView& groupView;
  TeachView& teachView;
  Lesson& lesson;
  TeachSession& session;

  QTableWidget* groupTable;

  int findRow(const Group*) const;

  friend class GroupTable;
};

class GroupTable : public QTableWidget {
  Q_OBJECT

public:
  GroupTable(Lesson& lesson, TeachSession& session) : lesson(lesson), session(session) {}

  Lesson& lesson;
  TeachSession& session;

  virtual void contextMenuEvent(QContextMenuEvent*);

private:
  std::vector<Group*> selectedGroups();

private slots:
  void selectAll();
  void selectCircles();
  void selectLines();
  void deleteGroup();
};
