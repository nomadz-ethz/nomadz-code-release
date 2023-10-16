/**
 * @file TeachWidget.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <unordered_map>
#include <QGraphicsScene>
#include <QImage>
#include <QLabel>
#include <QShortcut>
#include <QSlider>
#include <QTableWidget>
#include <QWidget>

#include "AnnotationItem.h"
#include "GroupCheckBox.h"
#include "PixmapTile.h"
#include "SimRobot.h"
#include "TeachView.h"
#include "TileGridView.h"
#include "Controller/Representations/Lesson.h"
#include "Controller/Views/TeachView/TeachSession.h"
#include "Representations/Infrastructure/Image.h"

/** Widget associated to TeachView to display pictures and stuff */
class TeachWidget : public QWidget, public SimRobot::Widget {
  Q_OBJECT;

public:
  TeachWidget(TeachView& teachView);

  virtual QMenu* createFileMenu() const { return teachView.createFileMenu(); }
  virtual QWidget* getWidget();

private slots:
  // TeachWidget -> Lesson
  void sendAnnotationMembership();
  // TeachWidget -> Annotator (Robot)
  void previewDataFrames(const std::vector<DataFrame*>&);
  // Lesson -> TeachWidget
  void updateGroupBar();
  void updateImagePage();
  void updateSelectedAnnotations(const std::unordered_set<Annotation*>&);
  // internal
  void handleSelections();

private:
  TeachView& teachView;
  Lesson& lesson;
  TeachSession& session;

  QLabel* imagePagePrompt;

  QWidget* imageTopBar;
  QSlider* columnsSlider;
  QWidget* groupBar;
  QLabel* imageCountLabel;
  QGraphicsScene* imageScene;
  TileGridView* imageView;

  std::unordered_map<GroupCheckBox*, Group*> checkboxToGroup;
  std::vector<QShortcut*> checkboxShortcuts;
  std::unordered_map<DataFrame*, PixmapTile*> frameToTile;
  std::unordered_map<Annotation*, AnnotationItem*> annotationToItem;

  void redrawImageView(const std::vector<DataFrame*>&, const std::vector<Group*>&);
};
