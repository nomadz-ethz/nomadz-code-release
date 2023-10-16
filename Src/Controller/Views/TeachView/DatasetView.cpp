/**
 * @file DatasetView.cpp
 *
 * Implementation of classes DatasetView & DatasetWidget
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <libgen.h>
#include <iostream>
#include <string>
#include <vector>
#include <QtGlobal>
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
#define setSectionResizeMode setResizeMode
#endif
#include <QCheckBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include "DatasetView.h"
#include "TeachView.h"
#include "Core/System/File.h"
#include "Core/System/Thread.h"
#include "Controller/ConsoleRoboCupCtrl.h"

DatasetView::DatasetView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView)
    : fullName(fullName), icon(":/Icons/tag_green.png"), console(console), teachView(teachView) {}

SimRobot::Widget* DatasetView::createWidget() {
  return new DatasetWidget(*this);
}

DatasetWidget::DatasetWidget(DatasetView& datasetView)
    : datasetView(datasetView), teachView(datasetView.teachView), lesson(datasetView.teachView.lesson),
      session(datasetView.teachView.session) {
  setFocusPolicy(Qt::StrongFocus);

  QVBoxLayout* layout = new QVBoxLayout(this);

  QHBoxLayout* loadBtnLayout = new QHBoxLayout();
  {
    QPushButton* loadLogBtn = new QPushButton("Load log");
    loadBtnLayout->addWidget(loadLogBtn);

    connect(loadLogBtn, SIGNAL(clicked()), this, SLOT(loadLog()));

    QPushButton* loadFolderBtn = new QPushButton("Load folder");
    loadBtnLayout->addWidget(loadFolderBtn);

    connect(loadFolderBtn, SIGNAL(clicked()), this, SLOT(loadFolder()));
  }
  layout->addLayout(loadBtnLayout);

  datasetTable = new DatasetTable();
  {
    datasetTable->setColumnCount(3);
    QStringList headerNames;
    headerNames << ""
                << "Dataset"
                << "Frames";
    datasetTable->setAlternatingRowColors(true);
    datasetTable->setHorizontalHeaderLabels(headerNames);
    datasetTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    datasetTable->horizontalHeader()->setStretchLastSection(true);
    datasetTable->verticalHeader()->setVisible(false);
    datasetTable->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    datasetTable->setAlternatingRowColors(true);
    datasetTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    datasetTable->setSortingEnabled(true);
    datasetTable->sortItems(NameCol, Qt::AscendingOrder);
    layout->addWidget(datasetTable);

    connect(datasetTable, SIGNAL(itemSelectionChanged()), this, SLOT(sendSelectedDatasets()));
    connect(datasetTable->horizontalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(sendVisibleDatasets()));
  }

  annotatorLayout = new QVBoxLayout();
  {
    robotBtn = new QPushButton(tr("Spawn virtual robot"));
    {
      annotatorLayout->addWidget(robotBtn);
      connect(robotBtn, SIGNAL(clicked()), this, SLOT(toggleAnnotator()));
    }

    annotateBtn = new QPushButton(tr("Annotate"));
    {
      annotateBtn->setDefault(true);
      annotateBtn->setDisabled(true);
      annotatorLayout->addWidget(annotateBtn);

      connect(annotateBtn, SIGNAL(clicked()), this, SLOT(startAnnotation()));
    }

    layout->addLayout(annotatorLayout);
  }

  connect(
    &lesson, SIGNAL(datasetsAdded(const std::vector<Dataset*>&)), this, SLOT(addDatasets(const std::vector<Dataset*>&)));
  connect(
    &lesson, SIGNAL(datasetsRemoved(const std::set<Dataset*>&)), this, SLOT(removeDatasets(const std::set<Dataset*>&)));
  connect(&lesson,
          SIGNAL(visibleDatasetsChanged(const std::vector<Dataset*>&)),
          this,
          SLOT(updateCheckboxes(const std::vector<Dataset*>&)));
  connect(&session, SIGNAL(annotatorChanged()), this, SLOT(updateAnnotatorStatus()));
}

QWidget* DatasetWidget::getWidget() {
  return this;
}

void DatasetWidget::loadLog() {
  using std::string;
  using std::to_string;
  string defaultDir = string(File::getBHDir()) + "/Config/Logs/";
  QString filename = QFileDialog::getOpenFileName(
    this, tr("Load log"), QString(File::getBHDir()) + "/Config/Logs/", tr("Robot log files (*.log)"));
  if (filename.isEmpty()) {
    return;
  }

  std::unique_ptr<Dataset> dataset = Dataset::loadLog(filename.toUtf8().constData());
  if (!dataset || dataset->frames.empty()) {
    return;
  }

  lesson.storeDataset(std::move(dataset));
}

void DatasetWidget::loadFolder() {
  QString dirname = QFileDialog::getExistingDirectory(
    this, tr("Load folder"), QString(File::getBHDir()) + "/Config/Logs/", QFileDialog::ShowDirsOnly);
  if (dirname.isEmpty()) {
    return;
  }

  std::unique_ptr<Dataset> dataset = Dataset::loadFolder(dirname.toUtf8().constData());
  if (!dataset || dataset->frames.empty()) {
    return;
  }

  lesson.storeDataset(std::move(dataset));
}

void DatasetWidget::sendSelectedDatasets() {
  std::vector<Dataset*> selection;

  auto selectedItems = datasetTable->selectedItems();
  for (auto& item : selectedItems) {
    selection.push_back(item->data(Qt::UserRole).value<Dataset*>());
  }

  session.selectDatasets(selection);

  annotateBtn->setEnabled(canAnnotate());
}

void DatasetWidget::addDatasets(const std::vector<Dataset*>& datasets) {
  datasetTable->setSortingEnabled(false);

  for (const auto& dataset : datasets) {
    datasetTable->insertRow(datasetTable->rowCount());
    const int row = datasetTable->rowCount() - 1;
    {
      QWidget* checkboxWrapper = new QWidget();
      {
        QHBoxLayout* checkboxLayout = new QHBoxLayout(checkboxWrapper);
        checkboxLayout->setAlignment(Qt::AlignCenter);
        QCheckBox* checkbox = new QCheckBox();
        {
          checkbox->setContentsMargins(0, 0, 0, 0);
          connect(checkbox, SIGNAL(stateChanged(int)), this, SLOT(sendVisibleDatasets()));
        }
        checkboxLayout->addWidget(checkbox);
      }
      datasetTable->setCellWidget(row, VisibilityCol, checkboxWrapper);
    }
    {
      QTableWidgetItem* nameCol = new QTableWidgetItem(dataset->name.c_str());
      nameCol->setData(Qt::UserRole, QVariant::fromValue(dataset));
      nameCol->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      datasetTable->setItem(row, NameCol, nameCol);
    }
    {
      QTableWidgetItem* framesCol = new QTableWidgetItem(QString::number(dataset->frames.size()));
      framesCol->setData(Qt::UserRole, QVariant::fromValue(dataset));
      framesCol->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      datasetTable->setItem(row, FramesCol, framesCol);
    }
  }

  datasetTable->setSortingEnabled(true);
}

void DatasetWidget::removeDatasets(const std::set<Dataset*>& datasets) {
  datasetTable->setSortingEnabled(false);

  for (int row = 0, rowCount = datasetTable->rowCount(); row < rowCount; ++row) {
    Dataset* dataset = datasetTable->item(row, NameCol)->data(Qt::UserRole).value<Dataset*>();
    if (datasets.find(dataset) != datasets.end()) {
      datasetTable->removeRow(row);
      --row;
      --rowCount;
    }
  }

  datasetTable->setSortingEnabled(true);
}

void DatasetWidget::sendVisibleDatasets() {
  std::vector<Dataset*> shown;

  for (int row = 0, rowCount = datasetTable->rowCount(); row < rowCount; ++row) {
    if (datasetTable->cellWidget(row, VisibilityCol)->findChild<QCheckBox*>()->isChecked()) {
      shown.push_back(datasetTable->item(row, NameCol)->data(Qt::UserRole).value<Dataset*>());
    }
  }

  lesson.showDatasets(shown);
}

void DatasetWidget::updateCheckboxes(const std::vector<Dataset*>& visibleDatasets) {
  std::unordered_set<Dataset*> lookup(visibleDatasets.begin(), visibleDatasets.end());

  for (int row = 0, rowCount = datasetTable->rowCount(); row < rowCount; ++row) {
    Dataset* dataset = datasetTable->item(row, NameCol)->data(Qt::UserRole).value<Dataset*>();
    QCheckBox* checkbox = datasetTable->cellWidget(row, VisibilityCol)->findChild<QCheckBox*>();

    // Check box only if it is present in visibleDatasets
    const bool checked = (lookup.find(dataset) != lookup.end());

    // And block signals to prevent circularly updating Lesson & vice versa
    if (checkbox->isChecked() != checked) {
      const bool wasBlocked = checkbox->blockSignals(true);
      checkbox->setChecked(checked);
      checkbox->blockSignals(wasBlocked);
    }
  }
}

void DatasetWidget::toggleAnnotator() {
  if (!session.annotator) {
    session.storeAnnotator(new RoboticAnnotator(teachView.console));
    dynamic_cast<RoboticAnnotator&>(*session.annotator).spawnRobot();
    // TODO In ConsoleRoboCupCtrl, removing views related to robot from scene graph makes connected widgets crash
    // } else {
    //   teachView.console.removeStudentRobot(session.robot);
    //   session.setRobot(nullptr);
  }
}

void DatasetWidget::startAnnotation() {
  assert(canAnnotate());

  std::vector<DataFrame*> selectedFrames;

  // Prepare vector of DataFrames to annotate
  for (auto i = session.selectedDatasets.cbegin(); i != session.selectedDatasets.cend(); ++i) {
    const auto& frames = lesson.datasets[*i]->frames;
    for (auto j = frames.cbegin(); j != frames.cend(); ++j) {
      selectedFrames.push_back(j->second.get());
    }
  }

  if (session.annotator->start(selectedFrames)) {
    annotateBtn->setDisabled(true);
    annotateBtn->setText(tr("Annotating..."));
    connect(session.annotator.get(),
            SIGNAL(progressed(int, int)),
            this,
            SLOT(updateAnnotationProgress(int, int)),
            Qt::QueuedConnection);
    connect(session.annotator.get(), SIGNAL(finished()), this, SLOT(sendAnnotatorResults()), Qt::QueuedConnection);
  }
}

void DatasetWidget::updateAnnotationProgress(int value, int max) {
  annotateBtn->setText(tr("Annotating... (%1 / %2)").arg(value).arg(max));
}

void DatasetWidget::sendAnnotatorResults() {
  std::cout << "DatasetWidget::sendAnnotatorResults enter" << std::endl;
  disconnect(session.annotator.get(), SIGNAL(progressed(int, int)), this, SLOT(updateAnnotationProgress(int, int)));
  disconnect(session.annotator.get(), SIGNAL(finished()), this, SLOT(sendAnnotatorResults()));

  annotateBtn->setEnabled(true);
  annotateBtn->setText(tr("Annotate"));

  std::vector<std::unique_ptr<Annotation>> newAnnotations = session.annotator->giveResults();
  if (newAnnotations.empty()) {
    std::cout << "DatasetWidget::sendAnnotatorResults exit: newAnnotations.empty()" << std::endl;
    return;
  }

  // Make new group
  std::vector<std::unique_ptr<Annotation>> circles;
  std::vector<std::unique_ptr<Annotation>> lines;
  std::vector<std::unique_ptr<Annotation>> polygons;
  while (newAnnotations.size() > 0) {
    std::unique_ptr<Annotation>& annotation = newAnnotations.back();
    if (annotation->info.type() == typeid(Annotation::Circle)) {
      circles.push_back(std::move(annotation));
    } else if (annotation->info.type() == typeid(Annotation::Polygon)) {
      const Annotation::Polygon& polygon = boost::get<Annotation::Polygon>(annotation->info);
      if (polygon.points.size() == 2) {
        lines.push_back(std::move(annotation));
      } else {
        polygons.push_back(std::move(annotation));
      }
    }
    newAnnotations.pop_back();
  }

  std::cout << "DatasetWidget::sendAnnotatorResults making new groups with " << circles.size() << " circles, "
            << lines.size() << " lines" << std::endl;

  if (!circles.empty()) {
    lesson.storeAnnotationsAndGroup(std::move(circles), std::unique_ptr<Group>(new Group("circles")), true);
  }
  if (!lines.empty()) {
    lesson.storeAnnotationsAndGroup(std::move(lines), std::unique_ptr<Group>(new Group("lines")), true);
  }
  if (!polygons.empty()) {
    lesson.storeAnnotationsAndGroup(std::move(polygons), std::unique_ptr<Group>(new Group("polygons")), true);
  }

  // lesson.storeAnnotationsAndGroup(std::move(newAnnotations), std::unique_ptr<Group>(new Group("annotations")), true);
}

void DatasetWidget::updateAnnotatorStatus() {
  if (session.annotator) {
    robotBtn->setDisabled(true);
    robotBtn->setText(tr("Remove virtual robot"));
    robotBtn->setToolTip(tr("I'm sorry Dave, I'm afraid I can't do that."));
  } else {
    robotBtn->setText(tr("Spawn virtual robot"));
  }

  annotateBtn->setEnabled(canAnnotate());
}

bool DatasetWidget::canAnnotate() {
  return session.annotator && !session.selectedDatasets.empty();
}

void DatasetTable::contextMenuEvent(QContextMenuEvent* event) {
  QMenu menu;
  auto clickedCell = itemAt(event->pos());
  if (!clickedCell) {
    return;
  }

  event->accept();

  int clickedRow = row(clickedCell);
  Dataset* dataset = item(clickedRow, DatasetWidget::NameCol)->data(Qt::UserRole).value<Dataset*>();
  assert(dataset);

  QAction* saveToFolderAct = menu.addAction(tr("Save '%1' to folder...").arg(dataset->name.c_str()));
  saveToFolderAct->setData(QVariant::fromValue(dataset));
  connect(saveToFolderAct, SIGNAL(triggered()), this, SLOT(saveToFolder()));

  QAction* saveSomeToFolderAct = menu.addAction(tr("Save some of '%1' to folder...").arg(dataset->name.c_str()));
  saveSomeToFolderAct->setData(QVariant::fromValue(dataset));
  connect(saveSomeToFolderAct, SIGNAL(triggered()), this, SLOT(saveSomeToFolder()));

  menu.exec(event->globalPos());
}

void DatasetTable::saveToFolder() {
  Dataset* dataset = qobject_cast<QAction*>(sender())->data().value<Dataset*>();
  assert(dataset);

  QString dir = QFileDialog::getExistingDirectory(this,
                                                  tr("Save '%1' to folder").arg(dataset->name.c_str()),
                                                  QString(File::getBHDir()) + "/Config/Logs/",
                                                  QFileDialog::ShowDirsOnly);

  if (dir.isEmpty()) {
    return;
  }

  dir = dir + "/" + dataset->name.c_str();

  try {
    dataset->saveToFolder(dir.toUtf8().constData());
  } catch (std::runtime_error& e) {
    QMessageBox::critical(
      this, tr("Could not save to folder"), tr("Failed to save '%1' to '%2': %3").arg(dataset->name.c_str(), dir, e.what()));
  }
}

void DatasetTable::saveSomeToFolder() {
  Dataset* dataset = qobject_cast<QAction*>(sender())->data().value<Dataset*>();
  assert(dataset);

  bool ok;
  int interval = QInputDialog::getInt(this,
                                      tr("Save some of '%1' to folder").arg(dataset->name.c_str()),
                                      tr("Keep one frame every how many frames?\n(1 means keep everything)"),
                                      4,
                                      1,
                                      2147483647,
                                      1,
                                      &ok);

  if (!ok) {
    return;
  }

  const QString title = (interval == 1)
                          ? tr("Save '%1' to folder").arg(dataset->name.c_str())
                          : tr("Save 1 frame in %1 of '%2' to folder").arg(interval).arg(dataset->name.c_str());

  QString dir =
    QFileDialog::getExistingDirectory(this, title, QString(File::getBHDir()) + "/Config/Logs/", QFileDialog::ShowDirsOnly);

  if (dir.isEmpty()) {
    return;
  }

  dir = dir + "/" + dataset->name.c_str();

  // Make a copy of frames to keep
  int idx = -1;
  Dataset toKeep;
  for (const auto& i : dataset->frames) {
    ++idx;
    if (idx % interval != 0) {
      continue;
    }

    DataFrame* frame = i.second.get();

    std::unique_ptr<DataFrame> frame2(new DataFrame(*frame));
    toKeep.frames.insert(std::make_pair(frame->time, std::move(frame2)));
  }

  try {
    toKeep.saveToFolder(dir.toUtf8().constData());
  } catch (std::runtime_error& e) {
    QMessageBox::critical(
      this, tr("Could not save to folder"), tr("Failed to save '%1' to '%2': %3").arg(dataset->name.c_str(), dir, e.what()));
  }
}
