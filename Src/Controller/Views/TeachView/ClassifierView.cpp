/**
 * @file ClassifierView.cpp
 *
 * Implementation of classes ClassifierView & ClassifierWidget
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <string>
#include <vector>
#include <QtGlobal>
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
#define setSectionResizeMode setResizeMode
#endif
#include <QDir>
#include <QFileSystemWatcher>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include "ClassifierView.h"
#include "TeachView.h"
#include "Core/System/File.h"
#include "Core/System/Thread.h"
#include "Controller/ConsoleRoboCupCtrl.h"

ClassifierView::ClassifierView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView)
    : fullName(fullName), icon(":/Icons/tag_green.png"), console(console), teachView(teachView) {}

SimRobot::Widget* ClassifierView::createWidget() {
  return new ClassifierWidget(*this);
}

ClassifierWidget::ClassifierWidget(ClassifierView& classifierView)
    : classifierView(classifierView), teachView(classifierView.teachView), lesson(classifierView.teachView.lesson),
      session(classifierView.teachView.session),
      forestsPath(std::string(File::getBHDir()) + "/Config/Locations/Default/RandomForests") {
  setFocusPolicy(Qt::StrongFocus);

  QVBoxLayout* layout = new QVBoxLayout(this);

  {
    QString shortPath = QString(forestsPath.c_str());
    shortPath.remove(QString(File::getBHDir()));

    // TODO Elide label text, preferably keeping the right readable
    layout->addWidget(new QLabel(shortPath));
  }

  classifierTable = new QTableWidget();
  {
    classifierTable->setColumnCount(2);
    QStringList headerNames;
    headerNames << "Name"
                << "Type";
    classifierTable->setHorizontalHeaderLabels(headerNames);
    classifierTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    classifierTable->horizontalHeader()->setStretchLastSection(true);
    classifierTable->verticalHeader()->setVisible(false);
    classifierTable->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    classifierTable->setAlternatingRowColors(true);
    classifierTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    classifierTable->setSelectionMode(QAbstractItemView::SingleSelection);
    classifierTable->setSortingEnabled(true);
    classifierTable->sortItems(NameCol, Qt::AscendingOrder); // initial sort by increasing dataset name
    layout->addWidget(classifierTable);

    connect(classifierTable, SIGNAL(itemSelectionChanged()), this, SLOT(updateClassifyBtn()));

    connect(&session,
            SIGNAL(classifiersAdded(const std::vector<Classifier*>&)),
            this,
            SLOT(addClassifiers(const std::vector<Classifier*>&)));
    connect(&session,
            SIGNAL(classifiersRemoved(const std::set<Classifier*>&)),
            this,
            SLOT(removeClassifiers(const std::set<Classifier*>&)));
  }

  QFileSystemWatcher* watcher = new QFileSystemWatcher(this);
  {
    watcher->addPath(forestsPath.c_str());
    connect(watcher, SIGNAL(directoryChanged(const QString&)), this, SLOT(sendClassifiers()));
    sendClassifiers();
  }

  QHBoxLayout* groupLayout = new QHBoxLayout();
  {
    groupList = new QComboBox();
    {
      groupLayout->addWidget(groupList);

      connect(&lesson, SIGNAL(groupsAdded(const std::vector<Group*>&)), this, SLOT(updateGroups()));
      connect(&lesson, SIGNAL(groupsRemoved(const std::set<Group*>&)), this, SLOT(updateGroups()));
      connect(&lesson, SIGNAL(groupChanged(Group*)), this, SLOT(updateGroups()));
    }

    classifyBtn = new QPushButton(tr("Classify"));
    {
      groupLayout->addWidget(classifyBtn);

      connect(classifyBtn, SIGNAL(clicked()), this, SLOT(startClassification()));
      connect(&lesson, SIGNAL(groupChanged(Group*)), this, SLOT(updateClassifyBtn()));
      connect(groupList, SIGNAL(currentIndexChanged(int)), this, SLOT(updateClassifyBtn()));
    }

    layout->addLayout(groupLayout);
  }

  this->setLayout(layout);

  connect(&workerThread, SIGNAL(started()), this, SLOT(updateClassifyBtn()));
  connect(&workerThread, SIGNAL(finished()), this, SLOT(updateClassifyBtn()));
  connect(&workerThread, SIGNAL(terminated()), this, SLOT(updateClassifyBtn()));
}

ClassifierWidget::~ClassifierWidget() {
  workerThread.quit();
  workerThread.wait();
}

QWidget* ClassifierWidget::getWidget() {
  return this;
}

void ClassifierWidget::sendClassifiers() {
  QDir dir(forestsPath.c_str());
  const auto qPaths = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  std::vector<std::string> paths;
  for (const auto& qPath : qPaths) {
    paths.push_back(forestsPath + "/" + qPath.toUtf8().constData());
  }
  session.updateClassifiersInDir(forestsPath, paths);
}

void ClassifierWidget::addClassifiers(const std::vector<Classifier*>& classifiers) {
  classifierTable->setSortingEnabled(false);

  for (const auto& classifier : classifiers) {
    classifierTable->insertRow(classifierTable->rowCount());
    const int row = classifierTable->rowCount() - 1;

    {
      QTableWidgetItem* nameCol = new QTableWidgetItem();
      nameCol->setData(Qt::UserRole, QVariant::fromValue(classifier));
      nameCol->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      classifierTable->setItem(row, NameCol, nameCol);
    }

    {
      QTableWidgetItem* typeCol = new QTableWidgetItem();
      typeCol->setData(Qt::UserRole, QVariant::fromValue(classifier));
      typeCol->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      classifierTable->setItem(row, TypeCol, typeCol);
    }

    updateClassifier(classifier);
    connect(classifier, SIGNAL(changed(Classifier*)), this, SLOT(updateClassifier(Classifier*)));
  }

  classifierTable->setSortingEnabled(true);
}

void ClassifierWidget::removeClassifiers(const std::set<Classifier*>& classifiers) {
  classifierTable->setSortingEnabled(false);

  for (int row = 0, rowCount = classifierTable->rowCount(); row < rowCount; ++row) {
    Classifier* classifier = classifierTable->item(row, NameCol)->data(Qt::UserRole).value<Classifier*>();
    if (classifiers.find(classifier) != classifiers.end()) {
      classifierTable->removeRow(row);
      --row;
      --rowCount;
    }
  }

  classifierTable->setSortingEnabled(true);
}

int ClassifierWidget::findRow(const Classifier* classifier) const {
  if (!classifier) {
    return -1;
  }

  for (int row = 0, rowCount = classifierTable->rowCount(); row < rowCount; ++row) {
    if (classifierTable->item(row, NameCol)->data(Qt::UserRole).value<Classifier*>() == classifier) {
      return row;
    }
  }

  return -1;
}

void ClassifierWidget::updateClassifier(Classifier* classifier) {
  const int row = findRow(classifier);
  if (row == -1) {
    return;
  }

  {
    QString name = classifier->name().c_str();
    if (!classifier->empty() && !classifier->isSaved()) {
      name = name + "*";
    }
    classifierTable->item(row, NameCol)->setText(name);
  }

  {
    QString type = classifier->type().c_str();
    classifierTable->item(row, TypeCol)->setText(type);
  }
}

void ClassifierWidget::updateGroups() {
  std::vector<Group*> groups;
  for (const auto& i : lesson.groups) {
    groups.push_back(i.first);
  }
  groups.push_back(&lesson.ungrouped);

  // Sort by ascending name
  std::sort(groups.begin(), groups.end(), [](Group* a, Group* b) { return a->name < b->name; });

  groupList->clear();
  for (const auto& group : groups) {
    const int row = groupList->count();

    const int colorBoxSize = 32;
    QPixmap pixmap(colorBoxSize, colorBoxSize);
    { pixmap.fill(group->color); }

    const std::string itemText = group->name + " (" + std::to_string(group->members.size()) + ")";
    groupList->insertItem(row, QIcon(pixmap), itemText.c_str(), QVariant::fromValue(group));
  }
}

void ClassifierWidget::updateClassifyBtn() {
  Group* group = groupList->itemData(groupList->currentIndex()).value<Group*>();

  const bool canClassify =
    (classifierTable->selectedItems().size() > 0) && group && !(group->members.empty()) && !workerThread.isRunning();
  classifyBtn->setEnabled(canClassify);
}

void ClassifierWidget::startClassification() {
  Group* group = groupList->itemData(groupList->currentIndex()).value<Group*>();
  if (!group) {
    return;
  }

  if (classifierTable->selectedItems().size() <= 0) {
    return;
  }
  Classifier* classifier = classifierTable->selectedItems().first()->data(Qt::UserRole).value<Classifier*>();
  if (!classifier) {
    return;
  }

  QString forestPath = classifier->path().c_str();

  std::cout << "classify " << group->name << " using " << forestPath.toUtf8().constData() << std::endl;

  // Sample patches out of each annotation in the group
  std::vector<Annotation*> annotations(group->members.begin(), group->members.end());

  // TODO Make sampler user-selectable, and save user choice in Lesson or TeachSession
  std::unique_ptr<Sampler> sampler(new SinglePatchSampler(annotations));
  sampler->run();
  samples = sampler->giveResults();

  // Fill up the input vector
  std::unordered_map<Annotation*, std::vector<Sample*>> samplesByAnnotation;
  for (const auto& i : samples) {
    Sample* sample = i.get();
    Annotation* annotation = sample->annotation;

    if (samplesByAnnotation.find(annotation) == samplesByAnnotation.end()) {
      samplesByAnnotation[annotation] = {sample};
    } else {
      samplesByAnnotation[annotation].push_back(sample);
    }
  }

  std::cout << "Have " << samples.size() << " samples from " << samplesByAnnotation.size() << " annotations to classify"
            << std::endl;

  // Now get ready to classify
  classifier->load(
    classifier
      ->path()); // TODO HACK bin tree classifiers cannot be loaded in a separate thread! streaming doesn't work there.
  ClassificationWorker* worker = new MeanClassificationWorker(*classifier, samplesByAnnotation);

  worker->moveToThread(&workerThread);

  // These all get disconnected when worker is destroyed
  connect(&workerThread, SIGNAL(finished()), worker, SLOT(deleteLater()));
  connect(&workerThread, SIGNAL(started()), worker, SLOT(start()));
  connect(worker, SIGNAL(progressed(int, int)), this, SLOT(updateClassificationProgress(int, int)));
  connect(worker, SIGNAL(finished()), this, SLOT(sendClassifierResults()));

  workerThread.start();

  std::cout << "launched classification thread" << std::endl;
}

void ClassifierWidget::updateClassificationProgress(int val, int max) {
  classifyBtn->setText(tr("Classifying... (%1 / %2)").arg(val).arg(max));
}

void ClassifierWidget::sendClassifierResults() {
  ClassificationWorker* worker = qobject_cast<ClassificationWorker*>(sender());
  if (!worker) {
    std::cerr << "ClassifierWidget::sendClassifierResults: worker is null" << std::endl;
    return;
  }

  classifyBtn->setText(tr("Classify"));

  std::unordered_map<Annotation*, int> labels = worker->giveResults();
  std::cout << "sendClassifierResults: " << samples.size() << " labels" << std::endl;

  workerThread.quit();

  // Group annotations by label
  std::unordered_map<int, std::vector<Annotation*>> annotationsByLabel;
  for (const auto& i : labels) {
    Annotation* annotation = i.first;
    int label = i.second;

    if (annotationsByLabel.find(label) == annotationsByLabel.end()) {
      annotationsByLabel[label] = {annotation};
    } else {
      annotationsByLabel[label].push_back(annotation);
    }
  }

  // Find name of original group
  // TODO Fix: this actually gets the currently selected group, not necessarily the original one
  std::string originalName;
  {
    Group* group = groupList->itemData(groupList->currentIndex()).value<Group*>();
    if (group) {
      originalName = group->name;
    }
  }

  std::vector<std::unique_ptr<Group>> newGroups;

  // For each label, create a new group
  for (const auto& i : annotationsByLabel) {
    int label = i.first;
    const std::vector<Annotation*>& annotations = i.second;

    std::string newName = originalName + "/" + std::to_string(label);
    newGroups.emplace_back(new Group(newName, annotations));
  }

  lesson.storeGroups(std::move(newGroups));
}
