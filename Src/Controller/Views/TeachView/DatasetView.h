/**
 * @file DatasetView.h
 *
 * Declaration of classes DatasetView & DatasetWidget
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <string>
#include <unordered_map>
#include <QContextMenuEvent>
#include <QIcon>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include "SimRobot.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/Representations/Lesson.h"
#include "Controller/Views/TeachView/TeachSession.h"
#include "Controller/Views/TeachView/TeachView.h"

/**
 * @class DatasetView
 *
 * A view that shows the currently loaded datasets.
 */
class DatasetView : public SimRobot::Object {
public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param info The timing info object to be visualized.
   */
  DatasetView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView);

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

  friend class DatasetWidget; // DatasetWidget needs to access data stored in here
};

/**A widget that is used inside a DatasetView to display datasets*/
class DatasetWidget : public QWidget, public SimRobot::Widget {
  Q_OBJECT;

public:
  DatasetWidget(DatasetView& datasetView);

  virtual QMenu* createFileMenu() const { return teachView.createFileMenu(); }
  virtual QWidget* getWidget();

private slots:
  // DatasetWidget -> Lesson or TeachSession
  void loadLog();
  void loadFolder();
  void sendSelectedDatasets();
  void sendVisibleDatasets();
  void sendAnnotatorResults();
  // Lesson -> DatasetWidget
  void addDatasets(const std::vector<Dataset*>&);
  void removeDatasets(const std::set<Dataset*>&);
  void updateCheckboxes(const std::vector<Dataset*>&);
  // DatasetWidget -> filesystem
  // void saveFolder();
  // internal
  void toggleAnnotator();
  void startAnnotation();
  void updateAnnotatorStatus();
  void updateAnnotationProgress(int, int);

private:
  enum TableCol { VisibilityCol, NameCol, FramesCol };

  DatasetView& datasetView;
  TeachView& teachView;
  Lesson& lesson;
  TeachSession& session;

  int test = 0;

  QTableWidget* datasetTable;
  QPushButton* annotateBtn;

  QVBoxLayout* annotatorLayout;
  QPushButton* robotBtn;

  bool canAnnotate();

  friend class DatasetTable;
};

class DatasetTable : public QTableWidget {
  Q_OBJECT

public:
  virtual void contextMenuEvent(QContextMenuEvent*);

private slots:
  void saveToFolder();
  void saveSomeToFolder();
};
