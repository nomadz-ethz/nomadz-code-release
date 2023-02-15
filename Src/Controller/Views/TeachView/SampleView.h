/**
 * @file SampleView.h
 *
 * Declaration of classes SampleView & SampleWidget
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <string>
#include <unordered_map>
#include <QComboBox>
#include <QContextMenuEvent>
#include <QGraphicsScene>
#include <QIcon>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QThread>
#include "SimRobot.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/Representations/Lesson.h"
#include "Controller/Views/TeachView/TeachSession.h"
#include "Controller/Views/TeachView/TeachView.h"
#include "Controller/Views/TeachView/TileGridView.h"

/**
 * @class SampleView
 *
 * A view that shows the existing samples.
 */
class SampleView : public SimRobot::Object {
public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param info The timing info object to be visualized.
   */
  SampleView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView);

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

  friend class SampleWidget; // SampleWidget needs to access data stored in here
};

/**A widget that is used inside a SampleView to display groups*/
class SampleWidget : public QWidget, public SimRobot::Widget {
  Q_OBJECT;

public:
  SampleWidget(SampleView& sampleView);
  ~SampleWidget();

  virtual QMenu* createFileMenu() const { return teachView.createFileMenu(); }
  virtual QWidget* getWidget();

  virtual void contextMenuEvent(QContextMenuEvent*);

private slots:
  void startSampler();
  void showSamplerResults();
  void saveToFolder();

private:
  SampleView& sampleView;
  TeachView& teachView;
  Lesson& lesson;
  TeachSession& session;

  QLabel* sampleCountLabel;
  QGraphicsScene* sampleScene;
  TileGridView* sampleGrid;

  std::vector<std::unique_ptr<Sample>> samples;

  QThread samplerThread;

  std::string generateName();
};
