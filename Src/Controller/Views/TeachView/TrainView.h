/**
 * @file TrainView.h
 *
 * Declaration of classes TrainView & TrainWidget
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
#include <QThread>
#include "SimRobot.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/Representations/Lesson.h"
#include "Controller/Views/TeachView/Classifier.h"
#include "Controller/Views/TeachView/Sampler.h"
#include "Controller/Views/TeachView/TeachSession.h"
#include "Controller/Views/TeachView/TeachView.h"

/**
 * @class TrainView
 *
 * A view that shows the currently loaded groups.
 */
class TrainView : public SimRobot::Object {
public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param info The timing info object to be visualized.
   */
  TrainView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView);

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

  friend class TrainWidget; // TrainWidget needs to access data stored in here
};

/**A widget that is used inside a TrainView to display groups*/
class TrainWidget : public QWidget, public SimRobot::Widget {
  Q_OBJECT;

public:
  TrainWidget(TrainView& trainView);

  virtual QMenu* createFileMenu() const { return teachView.createFileMenu(); }
  virtual QWidget* getWidget();

  // TODO Combine this with what's in ClassifierView
  const std::string forestsPath;

private slots:
  // TrainWidget -> Lesson
  void sendEditedGroup(QTableWidgetItem*);

  // TrainWidget -> filesystem
  void saveTrainingResults();

  // Lesson -> TrainWidget
  void addGroups(const std::vector<Group*>&);
  void removeGroups(const std::set<Group*>&);
  void updateGroup(Group*);

  // internal
  void updateTrainBtn();
  void updateTrainingProgress(int, int);
  void startTraining();

private:
  enum TableCol { ClassCol, NameCol, SizeCol };

  TrainView& trainView;
  TeachView& teachView;
  Lesson& lesson;
  TeachSession& session;

  QLineEdit* newName;
  QTableWidget* groupTable;
  QPushButton* trainBtn;

  // Own the samples we currently happen to be training
  std::map<int, std::vector<std::unique_ptr<Sample>>> samplePtrsByClass;
  std::map<int, std::vector<Sample*>> samplesByClass;

  QThread workerThread;

  int findRow(const Group*) const;
  std::map<int, std::vector<Group*>> getGroupsByClass() const;
  std::map<int, std::vector<Annotation*>> getAnnotationsByClass() const;

  class ClassCell : public QTableWidgetItem {
  public:
    bool operator<(const QTableWidgetItem& other) const override;
  };
};

class TrainingWorker : public QObject {
public:
  TrainingWorker(const std::map<int, std::vector<Sample*>>& samples) : inputs(samples) {}
  virtual ~TrainingWorker(){};

  virtual Classifier* giveResults() = 0;

public slots:
  virtual void start() = 0;

signals:
  void finished();
  void progressed(int, int);

private:
  Q_OBJECT

protected:
  std::map<int, std::vector<Sample*>> inputs;
  Classifier* result;
};

class RandomForestTrainingWorker : public TrainingWorker {
public:
  RandomForestTrainingWorker(const std::map<int, std::vector<Sample*>>& samples) : TrainingWorker(samples) {}

  Classifier* giveResults() override;

  void setParameters();

  void start() override;
};
