/**
 * @file TeachView.cpp
 *
 * Implementation of class TeachView
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <QFileDialog>
#include <QString>

#include "TeachView.h"
#include "TeachWidget.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Core/System/File.h"
#include "Core/Streams/InStreams.h"

// Thanks https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
static bool ends_with(const std::string& value, const std::string& ending) {
  if (ending.size() > value.size()) {
    return false;
  }
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

TeachView::TeachView(const QString& fullName, ConsoleRoboCupCtrl& console)
    : fullName(fullName), icon(":/Icons/allColors.png"), console(console), lesson(), session(lesson) {}

SimRobot::Widget* TeachView::createWidget() {
  return new TeachWidget(*this);
}

QMenu* TeachView::createFileMenu() const {
  QMenu* menu = new QMenu(tr("&File"));

  QAction* newAction = new QAction(QIcon(":/Icons/bike_new.png"), tr("&New Lesson"), menu);
  newAction->setStatusTip(tr("Create a new Lesson"));
  menu->addAction(newAction);
  connect(newAction, SIGNAL(triggered()), this, SLOT(newLesson()));

  QAction* loadAction = new QAction(QIcon(":/Icons/bike_open.png"), tr("&Open Lesson..."), menu);
  loadAction->setShortcut(QKeySequence(tr("Ctrl+Shift+O")));
  loadAction->setStatusTip(tr("Open an existing Lesson"));
  menu->addAction(loadAction);
  connect(loadAction, SIGNAL(triggered()), this, SLOT(loadLesson()));

  QAction* mergeAction = new QAction(QIcon(":/Icons/bike_open.png"), tr("&Merge Lesson..."), menu);
  mergeAction->setStatusTip(tr("Merge an existing Lesson with the current Lesson"));
  menu->addAction(mergeAction);
  connect(mergeAction, SIGNAL(triggered()), this, SLOT(mergeLesson()));

  QAction* saveAsAction = new QAction(QIcon(":/Icons/bike_save_as.png"), tr("Save Lesson &As..."), menu);
  saveAsAction->setShortcut(QKeySequence(tr("Ctrl+Shift+S")));
  saveAsAction->setStatusTip(tr("Save to a Lesson file using a new name"));
  menu->addAction(saveAsAction);
  connect(saveAsAction, SIGNAL(triggered()), this, SLOT(saveLessonAs()));

  return menu;
}

void TeachView::newLesson() {
  lesson.clear();
}

void TeachView::loadLesson() {
  std::string dirname = std::string(File::getBHDir()) + "/Config/Lessons/";
  QString filename =
    QFileDialog::getOpenFileName(&dialogOwner, tr("Load Lesson"), dirname.c_str(), tr("Lesson files (*.lesson)"));
  if (filename.isEmpty()) {
    return;
  }

  lesson.clear();

  return mergeLessonAt(filename);
}

void TeachView::mergeLesson() {
  std::string dirname = std::string(File::getBHDir()) + "/Config/Lessons/";
  QString filename =
    QFileDialog::getOpenFileName(&dialogOwner, tr("Merge Lesson"), dirname.c_str(), tr("Lesson files (*.lesson)"));
  if (filename.isEmpty()) {
    return;
  }

  return mergeLessonAt(filename);
}

void TeachView::mergeLessonAt(const QString& lessonPath) {
  std::string dirname = std::string(File::getBHDir()) + "/Config/Lessons/";

  StreamableLesson sLesson;
  InMapFile stream(lessonPath.toUtf8().constData());
  stream >> sLesson;

  while (true) {
    try {
      lesson.merge(sLesson);
      break;

    } catch (DatasetLoadError& e) {
      // Missing Dataset source; prompt for correction, then try again
      std::string wrongPath = e.dataset.sourcePath;

      QString correctedPath;
      switch (e.dataset.source) {
      case Dataset::Log: {
        correctedPath = QFileDialog::getOpenFileName(&dialogOwner,
                                                     tr("Select a replacement file for '%1'").arg(wrongPath.c_str()),
                                                     dirname.c_str(),
                                                     tr("Robot log files (*.log)"));
        break;
      }
      case Dataset::Folder: {
        correctedPath = QFileDialog::getExistingDirectory(&dialogOwner,
                                                          tr("Select a replacement folder for '%1'").arg(wrongPath.c_str()),
                                                          dirname.c_str(),
                                                          QFileDialog::ShowDirsOnly);
        break;
      }
      default:
        std::cerr << "TeachView::loadLesson Failed: invalid source type for dataset " << e.dataset.name << ": "
                  << e.dataset.source << std::endl;
        return;
      }

      if (correctedPath.isEmpty()) {
        return;
      }

      for (auto& dataset : sLesson.datasets) {
        if (dataset.source == e.dataset.source && dataset.sourcePath == wrongPath) {
          dataset.sourcePath = correctedPath.toUtf8().constData();
        }
      }

      continue;
    }
  }
}

void TeachView::saveLessonAs() {
  std::string dirname = std::string(File::getBHDir()) + "/Config/Lessons/";
  QString filename =
    QFileDialog::getSaveFileName(&dialogOwner, tr("Save Lesson As"), dirname.c_str(), tr("Lesson files (*.lesson)"));
  if (filename.isEmpty()) {
    return;
  }

  // TODO On Linux, when "x.lesson" already exists but the user wants to save to "x", the save dialog will NOT prompt for
  // confirmation.
  //      Fixing this will require actually instantiating a QFileDialog.
  if (!filename.endsWith(".lesson")) {
    filename += ".lesson";
  }

  StreamableLesson sLesson(lesson); // TODO Save sLesson to (filename)
  OutMapFile stream(filename.toUtf8().constData());
  stream << sLesson;
}

void TeachView::handleConsole(const std::string& command) {
  std::string lessonName = command;

  // Find path corresponding to lessonName
  if (lessonName != "") {
    std::string lessonPath = std::string(File::getBHDir()) + "/Config/Lessons/" + lessonName;
    if (!ends_with(lessonPath, ".lesson")) {
      lessonPath += ".lesson";
    }

    File lessonFile(lessonPath, "r", false);

    if (lessonFile.exists()) {
      InMapFile stream(lessonPath);
      StreamableLesson sLesson;
      stream >> sLesson;

      // TODO Handle "are you sure you want to delete everything"
      lesson.clear();
      lesson.merge(sLesson);

      // TODO Get rid of this once the tool is stable
      console.printLn("teach: Loaded " + lessonPath + " (" + std::to_string(lesson.datasets.size()) + " dataset" +
                      (lesson.datasets.size() == 1 ? "" : "s") + ", " + std::to_string(lesson.annotations.size()) +
                      " annotation" + (lesson.annotations.size() == 1 ? "" : "s") + ")");

    } else {
      console.printLn("teach: Lesson file " + lessonPath + " does not exist");
    }
  }
}
