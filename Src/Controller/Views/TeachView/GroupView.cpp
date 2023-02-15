/**
 * @file GroupView.cpp
 *
 * Implementation of classes GroupView & GroupWidget
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
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
#include <QColorDialog>
#include <QContextMenuEvent>
#include <QFont>
#include <QHeaderView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include "GroupView.h"
#include "TeachView.h"
#include "Core/System/File.h"
#include "Core/System/Thread.h"
#include "Controller/ConsoleRoboCupCtrl.h"

GroupView::GroupView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView)
    : fullName(fullName), icon(":/Icons/tag_green.png"), console(console), teachView(teachView) {}

SimRobot::Widget* GroupView::createWidget() {
  return new GroupWidget(*this);
}

GroupWidget::GroupWidget(GroupView& groupView)
    : groupView(groupView), teachView(groupView.teachView), lesson(groupView.teachView.lesson),
      session(groupView.teachView.session) {
  setFocusPolicy(Qt::StrongFocus);

  QVBoxLayout* layout = new QVBoxLayout(this);

  groupTable = new GroupTable(lesson, session);
  {
    groupTable->setColumnCount(4);
    QStringList headerNames;
    headerNames << ""
                << ""
                << "Group"
                << "Size";
    groupTable->setHorizontalHeaderLabels(headerNames);
    groupTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    groupTable->horizontalHeader()->setStretchLastSection(true);
    groupTable->verticalHeader()->setVisible(false);
    groupTable->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    groupTable->setAlternatingRowColors(true);
    groupTable->setEditTriggers(QAbstractItemView::DoubleClicked);
    groupTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    groupTable->setSortingEnabled(true);
    groupTable->sortItems(NameCol, Qt::AscendingOrder); // initial sort by increasing group name
    layout->addWidget(groupTable);

    connect(groupTable, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(handleDoubleClick(int, int)));
    connect(groupTable, SIGNAL(itemChanged(QTableWidgetItem*)), this, SLOT(sendEditedGroup(QTableWidgetItem*)));
    connect(groupTable, SIGNAL(itemSelectionChanged()), this, SLOT(sendSelectedGroups()));
    connect(groupTable->horizontalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(sendVisibleGroups()));

    addGroups({&lesson.ungrouped});
  }

  QPushButton* createGroupBtn = new QPushButton(tr("New group"));
  {
    layout->addWidget(createGroupBtn);
    connect(createGroupBtn, SIGNAL(clicked()), this, SLOT(createGroup()));
  }

  this->setLayout(layout);

  connect(&lesson, SIGNAL(groupsAdded(const std::vector<Group*>&)), this, SLOT(addGroups(const std::vector<Group*>&)));
  connect(&lesson, SIGNAL(groupsRemoved(const std::set<Group*>&)), this, SLOT(removeGroups(const std::set<Group*>&)));
  connect(&lesson,
          SIGNAL(visibleGroupsChanged(const std::vector<Group*>&)),
          this,
          SLOT(updateCheckboxes(const std::vector<Group*>&)));
}

QWidget* GroupWidget::getWidget() {
  return this;
}

void GroupWidget::addGroups(const std::vector<Group*>& groups) {
  groupTable->setSortingEnabled(false);

  for (const auto& group : groups) {
    groupTable->insertRow(groupTable->rowCount());
    const int row = groupTable->rowCount() - 1;
    {
      QWidget* checkboxWrapper = new QWidget();
      {
        QHBoxLayout* checkboxLayout = new QHBoxLayout(checkboxWrapper);
        checkboxLayout->setAlignment(Qt::AlignCenter);
        QCheckBox* checkbox = new QCheckBox();
        {
          checkbox->setContentsMargins(0, 0, 0, 0);
          connect(checkbox, SIGNAL(stateChanged(int)), this, SLOT(sendVisibleGroups()));
        }
        checkboxLayout->addWidget(checkbox);
      }
      groupTable->setCellWidget(row, VisibilityCol, checkboxWrapper);
    }
    {
      QWidget* colorboxWrapper = new QWidget();
      {
        QHBoxLayout* colorboxLayout = new QHBoxLayout(colorboxWrapper);
        colorboxLayout->setAlignment(Qt::AlignCenter);

        QLabel* colorbox = new QLabel();
        colorboxLayout->addWidget(colorbox);
      }
      groupTable->setCellWidget(row, ColorCol, colorboxWrapper);
    }
    {
      QTableWidgetItem* nameCol = new QTableWidgetItem();
      nameCol->setData(Qt::UserRole, QVariant::fromValue(group));
      if (group == &(lesson.ungrouped)) {
        nameCol->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      } else {
        nameCol->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      }

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

void GroupWidget::removeGroups(const std::set<Group*>& groups) {
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

void GroupWidget::createGroup() {
  std::unique_ptr<Group> group(new Group("new"));
  lesson.storeGroup(std::move(group));
}

void GroupWidget::handleDoubleClick(int row, int col) {
  Group* group = groupTable->item(row, NameCol)->data(Qt::UserRole).value<Group*>();
  switch (col) {
  case ColorCol: {
    QColor newColor = QColorDialog::getColor(group->color, this, tr("Choose a color for '%1'").arg(group->name.c_str()));
    if (newColor.isValid()) {
      group->setColor(newColor);
    }
    break;
  }
  case SizeCol: {
    session.selectAnnotations(group->members);
    break;
  }
  }
}

void GroupWidget::sendEditedGroup(QTableWidgetItem* item) {
  Group* group = item->data(Qt::UserRole).value<Group*>();
  if (item->column() == NameCol) {
    group->setName(item->text().toUtf8().constData());
  }
}

void GroupWidget::sendSelectedGroups() {
  std::vector<Group*> selection;

  auto selectedItems = groupTable->selectedItems();
  for (auto& item : selectedItems) {
    if (item->column() == NameCol) { // Make sure only done once per row
      selection.push_back(item->data(Qt::UserRole).value<Group*>());
    }
  }

  session.selectGroups(selection);
}

void GroupWidget::sendVisibleGroups() {
  std::vector<Group*> shown;

  for (int row = 0, rowCount = groupTable->rowCount(); row < rowCount; ++row) {
    if (groupTable->cellWidget(row, VisibilityCol)->findChild<QCheckBox*>()->isChecked()) {
      shown.push_back(groupTable->item(row, NameCol)->data(Qt::UserRole).value<Group*>());
    }
  }

  lesson.showGroups(shown);
}

void GroupWidget::updateCheckboxes(const std::vector<Group*>& visibleGroups) {
  std::unordered_set<Group*> lookup(visibleGroups.begin(), visibleGroups.end());

  for (int row = 0, rowCount = groupTable->rowCount(); row < rowCount; ++row) {
    Group* group = groupTable->item(row, NameCol)->data(Qt::UserRole).value<Group*>();
    QCheckBox* checkbox = groupTable->cellWidget(row, VisibilityCol)->findChild<QCheckBox*>();

    // Check box only if it is present in visibleGroups
    const bool checked = (lookup.find(group) != lookup.end());

    // And block signals to prevent circularly updating Lesson & vice versa
    if (checkbox->isChecked() != checked) {
      const bool wasBlocked = checkbox->blockSignals(true);
      checkbox->setChecked(checked);
      checkbox->blockSignals(wasBlocked);
    }
  }
}

int GroupWidget::findRow(const Group* group) const {
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

void GroupWidget::updateGroup(Group* group) {
  const int row = findRow(group);
  if (row == -1) {
    return;
  }

  const int colorBoxSize = 32;
  QPixmap pixmap(colorBoxSize, colorBoxSize);
  {
    pixmap.fill(group->color);
    groupTable->cellWidget(row, ColorCol)->findChild<QLabel*>()->setPixmap(pixmap);
  }

  const bool wasBlocked = groupTable->blockSignals(true);
  {
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

void GroupTable::contextMenuEvent(QContextMenuEvent* event) {
  QMenu menu;
  auto clickedCell = itemAt(event->pos());
  if (!clickedCell) {
    return;
  }

  int clickedRow = row(clickedCell);
  Group* clickedGroup = item(clickedRow, GroupWidget::NameCol)->data(Qt::UserRole).value<Group*>();
  assert(clickedGroup);

  const std::vector<Group*> groups = selectedGroups();

  event->accept();

  {
    const auto annotations = Group::getAll(groups);
    QAction* selectAllAct = menu.addAction(tr("Select &all (%1)").arg(annotations.size()));
    selectAllAct->setEnabled(!annotations.empty());
    connect(selectAllAct, SIGNAL(triggered()), this, SLOT(selectAll()));
  }

  {
    const auto annotations = Group::getCircles(groups);
    QAction* selectCirclesAct = menu.addAction(tr("Select &circles (%1)").arg(annotations.size()));
    selectCirclesAct->setEnabled(!annotations.empty());
    connect(selectCirclesAct, SIGNAL(triggered()), this, SLOT(selectCircles()));
  }

  {
    const auto annotations = Group::getLines(groups);
    QAction* selectLinesAct = menu.addAction(tr("Select &lines (%1)").arg(annotations.size()));
    selectLinesAct->setEnabled(!annotations.empty());
    connect(selectLinesAct, SIGNAL(triggered()), this, SLOT(selectLines()));
  }

  menu.addSeparator();

  QAction* deleteAct =
    menu.addAction(tr(clickedGroup->members.empty() ? "&Delete '%1'" : "&Delete '%1'...").arg(clickedGroup->name.c_str()));
  deleteAct->setData(QVariant::fromValue(clickedGroup));
  deleteAct->setEnabled(clickedGroup != &(lesson.ungrouped));
  connect(deleteAct, SIGNAL(triggered()), this, SLOT(deleteGroup()));

  menu.exec(event->globalPos());
}

std::vector<Group*> GroupTable::selectedGroups() {
  std::vector<Group*> groups;
  for (const auto& item : selectedItems()) {
    if (item->column() == GroupWidget::NameCol) {
      Group* group = item->data(Qt::UserRole).value<Group*>();
      if (group) {
        groups.push_back(group);
      }
    }
  }
  return groups;
}

void GroupTable::selectAll() {
  session.selectAnnotations(Group::getAll(selectedGroups()));
}

void GroupTable::selectCircles() {
  session.selectAnnotations(Group::getCircles(selectedGroups()));
}

void GroupTable::selectLines() {
  session.selectAnnotations(Group::getLines(selectedGroups()));
}

void GroupTable::deleteGroup() {
  Group* group = qobject_cast<QAction*>(sender())->data().value<Group*>();
  assert(group);

  const std::string prompt =
    (group->members.size() == 1)
      ? "Are you sure you want to delete '%1'?\n\n%2 annotation will no longer belong to this group. This cannot be undone."
      : "Are you sure you want to delete '%1'?\n\n%2 annotations will no longer belong to this group. This cannot be "
        "undone.";

  if (!group->members.empty() && QMessageBox::warning(this,
                                                      tr("Delete '%1'").arg(group->name.c_str()),
                                                      tr(prompt.c_str()).arg(group->name.c_str()).arg(group->members.size()),
                                                      QMessageBox::Yes | QMessageBox::No,
                                                      QMessageBox::No) == QMessageBox::No) {
    return;
  }

  lesson.deleteGroup(group);
}
