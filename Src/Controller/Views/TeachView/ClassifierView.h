/**
 * @file ClassifierView.h
 *
 * Declaration of classes ClassifierView & ClassifierWidget
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <string>
#include <unordered_map>
#include <QComboBox>
#include <QIcon>
#include <QPushButton>
#include <QTableWidget>
#include <QThread>
#include "SimRobot.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/Representations/Lesson.h"
#include "Controller/Views/TeachView/Classifier.h"
#include "Controller/Views/TeachView/Sampler.h"
#include "Controller/Views/TeachView/TeachSession.h"
#include "Controller/Views/TeachView/TeachView.h"

/**
 * @class ClassifierView
 *
 * A view that shows the existing classifiers.
 */
class ClassifierView : public SimRobot::Object {
public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param info The timing info object to be visualized.
   */
  ClassifierView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView);

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

  friend class ClassifierWidget; // ClassifierWidget needs to access data stored in here
};

/**A widget that is used inside a ClassifierView to display groups*/
class ClassifierWidget : public QWidget, public SimRobot::Widget {
  Q_OBJECT;

public:
  ClassifierWidget(ClassifierView& classifierView);
  ~ClassifierWidget();

  virtual QMenu* createFileMenu() const { return teachView.createFileMenu(); }
  virtual QWidget* getWidget();

  // TODO Combine this with what's in TrainView
  const std::string forestsPath;

private slots:
  // ClassifierWidget/filesystem -> TeachSession
  void sendClassifiers();
  // TeachSession -> ClassifierWidget
  void addClassifiers(const std::vector<Classifier*>&);
  void removeClassifiers(const std::set<Classifier*>&);
  void updateClassifier(Classifier*);

  // ClassifierWidget -> Lesson
  void sendClassifierResults();
  // Lesson -> ClassifierWidget
  // void updateClassifiers();
  void updateGroups();
  void updateClassifyBtn();
  void updateClassificationProgress(int, int);

  // internal?
  void startClassification();

private:
  enum TableCol { NameCol, TypeCol };

  ClassifierView& classifierView;
  TeachView& teachView;
  Lesson& lesson;
  TeachSession& session;

  QTableWidget* classifierTable;
  QComboBox* groupList;
  QPushButton* classifyBtn;

  std::vector<std::unique_ptr<Sample>> samples;

  QThread workerThread;

  int findRow(const Classifier*) const;
};
