/**
 * @file TrainView.cpp
 *
 * Implementation of classes TrainView & TrainWidget
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
#include <QFont>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRegExp>
#include <QString>
#include <QTime>
#include <QVBoxLayout>
#include "TeachView.h"
#include "TrainView.h"
#include "Core/System/File.h"
#include "Core/System/Thread.h"
#include "Controller/ConsoleRoboCupCtrl.h"

TrainView::TrainView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView)
    : fullName(fullName), icon(":/Icons/tag_green.png"), console(console), teachView(teachView) {}

SimRobot::Widget* TrainView::createWidget() {
  return new TrainWidget(*this);
}

TrainWidget::TrainWidget(TrainView& trainView)
    : trainView(trainView), teachView(trainView.teachView), lesson(trainView.teachView.lesson),
      session(trainView.teachView.session),
      forestsPath(std::string(File::getBHDir()) + "/Config/Locations/Default/RandomForests") {
  setFocusPolicy(Qt::StrongFocus);

  QVBoxLayout* layout = new QVBoxLayout(this);

  newName = new QLineEdit();
  {
    newName->setPlaceholderText(tr("New classifier name"));
    layout->addWidget(newName);

    // "Valid filenames" contain alphanumeric characters, dashes, underscores,
    // spaces, and periods; they must not be empty, begin with a space or dash,
    // have any character that is not a period or alphanumeric after the first
    // period, end with a period, or contain consecutive periods.
    // (This is probably more restrictive than necessary, but it's good enough.)
    QRegExp filenameRules("^[0-9A-Za-z_][0-9A-Za-z-_ ]*(\\.[0-9A-Za-z])*$");
    QValidator* validator = new QRegExpValidator(filenameRules, this);
    newName->setValidator(validator);

    connect(newName, SIGNAL(textEdited(const QString&)), this, SLOT(updateTrainBtn()));
  }

  groupTable = new QTableWidget();
  {
    groupTable->setColumnCount(3);
    QStringList headerNames;
    headerNames << tr("Class") << tr("Group") << tr("Size");
    groupTable->setHorizontalHeaderLabels(headerNames);
    groupTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    groupTable->horizontalHeader()->setStretchLastSection(true);
    groupTable->verticalHeader()->setVisible(false);
    groupTable->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    groupTable->setAlternatingRowColors(true);
    groupTable->setEditTriggers(QAbstractItemView::AllEditTriggers);
    groupTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    groupTable->setSortingEnabled(true);
    groupTable->sortItems(NameCol, Qt::AscendingOrder); // initial sort by increasing dataset name
    layout->addWidget(groupTable);

    connect(groupTable, SIGNAL(itemChanged(QTableWidgetItem*)), this, SLOT(updateTrainBtn()));

    addGroups({&lesson.ungrouped});
  }

  trainBtn = new QPushButton(tr("Train"));
  {
    layout->addWidget(trainBtn);

    connect(trainBtn, SIGNAL(clicked()), this, SLOT(startTraining()));

    updateTrainBtn();
  }

  this->setLayout(layout);

  connect(&lesson, SIGNAL(groupsAdded(const std::vector<Group*>&)), this, SLOT(addGroups(const std::vector<Group*>&)));
  connect(&lesson, SIGNAL(groupsRemoved(const std::set<Group*>&)), this, SLOT(removeGroups(const std::set<Group*>&)));

  connect(&workerThread, SIGNAL(started()), this, SLOT(updateTrainBtn()));
  connect(&workerThread, SIGNAL(finished()), this, SLOT(updateTrainBtn()));
  connect(&workerThread, SIGNAL(terminated()), this, SLOT(updateTrainBtn()));
}

QWidget* TrainWidget::getWidget() {
  return this;
}

void TrainWidget::addGroups(const std::vector<Group*>& groups) {
  groupTable->setSortingEnabled(false);

  for (const auto& group : groups) {
    groupTable->insertRow(groupTable->rowCount());
    const int row = groupTable->rowCount() - 1;
    {
      QTableWidgetItem* classCol = new ClassCell();
      classCol->setData(Qt::UserRole, QVariant::fromValue(group));
      classCol->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      classCol->setTextAlignment(Qt::AlignCenter);
      classCol->setToolTip(
        tr("The class in which to classify this group (integer, starting at 0). Multiple groups can share the same class."));

      const bool wasBlocked = groupTable->blockSignals(true);
      groupTable->setItem(row, ClassCol, classCol);
      groupTable->blockSignals(wasBlocked);
    }
    {
      QTableWidgetItem* nameCol = new QTableWidgetItem();
      nameCol->setData(Qt::UserRole, QVariant::fromValue(group));
      nameCol->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      const bool wasBlocked = groupTable->blockSignals(true);
      groupTable->setItem(row, NameCol, nameCol);
      groupTable->blockSignals(wasBlocked);
    }
    {
      QTableWidgetItem* sizeCol = new QTableWidgetItem();
      sizeCol->setData(Qt::UserRole, QVariant::fromValue(group));
      sizeCol->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      const bool wasBlocked = groupTable->blockSignals(true);
      groupTable->setItem(row, SizeCol, sizeCol);
      groupTable->blockSignals(wasBlocked);
    }

    updateGroup(group);
    connect(group, SIGNAL(changed(Group*)), this, SLOT(updateGroup(Group*)));
  }

  groupTable->setSortingEnabled(true);
}

void TrainWidget::removeGroups(const std::set<Group*>& groups) {
  groupTable->setSortingEnabled(false);

  for (int row = 0, rowCount = groupTable->rowCount(); row < rowCount; ++row) {
    Group* group = groupTable->item(row, NameCol)->data(Qt::UserRole).value<Group*>();
    if (groups.find(group) != groups.end()) {
      groupTable->removeRow(row);
      --row;
      --rowCount;
    }
  }

  groupTable->setSortingEnabled(true);
}

void TrainWidget::sendEditedGroup(QTableWidgetItem* item) {
  // Group* group = item->data(Qt::UserRole).value<Group*>();
  // if (item->column() == NameCol) {
  //   group->setName(item->text().toUtf8().constData());
  // }
}

void TrainWidget::updateTrainBtn() {
  const bool notTraining = !workerThread.isRunning();

  const bool nameValid = newName->hasAcceptableInput();

  const bool classInputValid = !getGroupsByClass().empty();

  trainBtn->setEnabled(notTraining && nameValid && classInputValid);

  if (!nameValid) {
    trainBtn->setToolTip(tr("Choose a valid name for the new classifier."));
  } else if (!classInputValid) {
    trainBtn->setToolTip(tr("Assign groups to valid classes to continue. Classes begin at 0 and go up one by one."));
  } else {
    trainBtn->setToolTip("");
  }
}

std::map<int, std::vector<Group*>> TrainWidget::getGroupsByClass() const {
  std::map<int, std::vector<Group*>> groupsByClass;

  // Get the class values from the table
  std::set<int> classes;
  for (int row = 0, rowCount = groupTable->rowCount(); row < rowCount; ++row) {
    bool isInt;
    const int c = groupTable->item(row, ClassCol)->text().toInt(&isInt);
    Group* group = groupTable->item(row, NameCol)->data(Qt::UserRole).value<Group*>();

    if (isInt && group) {
      classes.insert(c);

      if (groupsByClass.find(c) == groupsByClass.end()) {
        groupsByClass[c] = {group};
      } else {
        groupsByClass[c].push_back(group);
      }
    }
  }

  if (classes.empty()) {
    return {};
  }

  // Check classes start at 0 & go up one by one
  int idx = -1;
  for (const int c : classes) {
    if (c != ++idx) {
      return {};
    }
  }

  return groupsByClass;
}

std::map<int, std::vector<Annotation*>> TrainWidget::getAnnotationsByClass() const {
  std::map<int, std::vector<Annotation*>> annotationsByClass;

  std::map<int, std::vector<Group*>> groupsByClass = getGroupsByClass();
  for (const auto& i : groupsByClass) {
    int c = i.first;
    const std::vector<Group*>& groups = i.second;

    if (annotationsByClass.find(c) == annotationsByClass.end()) {
      annotationsByClass[c] = {};
    }

    for (Group* group : groups) {
      for (Annotation* annotation : group->members) {
        annotationsByClass[c].push_back(annotation);
      }
    }
  }

  return annotationsByClass;
}

int TrainWidget::findRow(const Group* group) const {
  if (!group) {
    return -1;
  }

  for (int row = 0, rowCount = groupTable->rowCount(); row < rowCount; ++row) {
    if (groupTable->item(row, NameCol)->data(Qt::UserRole).value<Group*>() == group) {
      return row;
    }
  }

  return -1;
}

void TrainWidget::updateGroup(Group* group) {
  const int row = findRow(group);
  if (row == -1) {
    return;
  }

  std::cout << "TrainWidget::updateGroup " << row << std::endl;

  const int colorBoxSize = 24;
  QPixmap pixmap(colorBoxSize, colorBoxSize);
  { pixmap.fill(group->color); }

  const bool wasBlocked = groupTable->blockSignals(true);
  {
    groupTable->item(row, NameCol)->setIcon(QIcon(pixmap));
    groupTable->item(row, NameCol)->setText(group->name.c_str());
    if (group == &(lesson.ungrouped)) {
      QFont font;
      font.setItalic(true);
      groupTable->item(row, NameCol)->setFont(font);
    }

    groupTable->item(row, SizeCol)->setText(QString::number(group->members.size()));
  }
  groupTable->blockSignals(wasBlocked);
}

void TrainWidget::startTraining() {
  std::cout << "TrainWidget::startTraining name: " << newName->text().toUtf8().constData() << std::endl;

  std::map<int, std::vector<Annotation*>> annotationsByClass = getAnnotationsByClass();

  QTime t;
  t.start();

  samplePtrsByClass.clear();
  samplesByClass.clear();
  // Sample annotations for each class
  for (const auto& i : annotationsByClass) {
    int c = i.first;
    const std::vector<Annotation*>& annotations = i.second;

    std::unique_ptr<Sampler> sampler(new SinglePatchSampler(annotations));
    sampler->run();
    samplePtrsByClass[c] = sampler->giveResults();

    std::vector<Sample*> samples;
    for (const std::unique_ptr<Sample>& ptr : samplePtrsByClass[c]) {
      samples.push_back(ptr.get());
    }
    samplesByClass[c] = samples;
  }

  std::cout << "startTraining: " << t.elapsed() << " ms to sample " << samplePtrsByClass.size() << " classes" << std::endl;

  // Now get ready to train
  TrainingWorker* worker = new RandomForestTrainingWorker(samplesByClass);

  worker->moveToThread(&workerThread);

  // These all get disconnected when worker is destroyed
  connect(&workerThread, SIGNAL(finished()), worker, SLOT(deleteLater()));
  connect(&workerThread, SIGNAL(started()), worker, SLOT(start()));
  connect(worker, SIGNAL(progressed(int, int)), this, SLOT(updateTrainingProgress(int, int)));
  connect(worker, SIGNAL(finished()), this, SLOT(saveTrainingResults()));

  workerThread.start();

  std::cout << "launched training thread" << std::endl;
}

void TrainWidget::updateTrainingProgress(int val, int max) {
  trainBtn->setText(tr("Training... (%1 / %2)").arg(val).arg(max));
}

void TrainWidget::saveTrainingResults() {
  TrainingWorker* worker = qobject_cast<TrainingWorker*>(sender());
  if (!worker) {
    std::cerr << "TrainWidget::saveTrainingResults: worker is null" << std::endl;
    return;
  }

  trainBtn->setText(tr("Train"));

  Classifier* classifier = worker->giveResults();

  // TODO Fix: this actually gets the current value in the newName box, not necessarily the original one
  const QString fullPath = QString(forestsPath.c_str()) + "/" + newName->text();
  classifier->save(fullPath.toUtf8().constData());

  workerThread.quit();
}

bool TrainWidget::ClassCell::operator<(const QTableWidgetItem& other) const {
  bool isInt1, isInt2;
  const int value1 = text().toInt(&isInt1);
  const int value2 = other.text().toInt(&isInt2);

  // Rank integers before anything else
  if (isInt1 && isInt2) {
    // Both are integers
    return value1 < value2;
  } else if (isInt1 || isInt2) {
    // One of them is integer
    return isInt1;
  } else {
    // Neither are integers
    return QTableWidgetItem::operator<(other);
  }
}

void RandomForestTrainingWorker::setParameters() {
  // TODO
}

void RandomForestTrainingWorker::start() {
  result = new OldRandomForestClassifier();
  emit progressed(0, 1);

  result->train(inputs);
  emit finished();
}

Classifier* RandomForestTrainingWorker::giveResults() {
  return result;
}
