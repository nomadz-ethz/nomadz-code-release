/**
 * @file TeachWidget.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <iostream>
#include <QButtonGroup>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPixmapCache>
#include <QPushButton>
#include <QStackedWidget>
#include <QTableWidgetItem>
#include <QTime>
#include <QVBoxLayout>

#include "TeachWidget.h"
#include "Core/System/File.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Representations/Infrastructure/Image.h"
#include "Core/Streams/OutStreams.h"

TeachWidget::TeachWidget(TeachView& teachView) : teachView(teachView), lesson(teachView.lesson), session(teachView.session) {
  setFocusPolicy(Qt::StrongFocus);

  QPixmapCache::setCacheLimit(400 * 1024); // 400 MB

  QVBoxLayout* layout = new QVBoxLayout(this);

  QButtonGroup* modeSwitcher = new QButtonGroup(this);
  QPushButton* imagePageBtn = new QPushButton(tr("Images"));
  imagePageBtn->setCheckable(true);
  imagePageBtn->setChecked(true);
  QPushButton* annotationPageBtn = new QPushButton(tr("Annotations"));
  annotationPageBtn->setCheckable(true);
  annotationPageBtn->setDisabled(true);
  annotationPageBtn->setToolTip(tr("To be implemented"));
  modeSwitcher->addButton(imagePageBtn, 0);
  modeSwitcher->addButton(annotationPageBtn, 1);
  QHBoxLayout* topLayout = new QHBoxLayout();
  topLayout->addWidget(modeSwitcher->button(0));
  topLayout->addWidget(modeSwitcher->button(1));
  // TODO This is not yet implemented and just takes space; uncomment when Annotations mode is ready
  // layout->addLayout(topLayout);

  QStackedWidget* pages = new QStackedWidget();
  {

    QWidget* imagePage = new QWidget();
    {
      pages->addWidget(imagePage);

      QVBoxLayout* imageLayout = new QVBoxLayout();
      imagePage->setLayout(imageLayout);

      imagePagePrompt = new QLabel(tr("Placeholder"));
      {
        imagePagePrompt->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        imageLayout->addWidget(imagePagePrompt);
      }

      imageTopBar = new QWidget();
      {
        QHBoxLayout* imageTopBarLayout = new QHBoxLayout();
        imageTopBar->setLayout(imageTopBarLayout);

        columnsSlider = new QSlider(Qt::Horizontal, this);
        {
          columnsSlider->setInvertedAppearance(true);
          columnsSlider->setMaximum(9);
          columnsSlider->setMinimum(1);
          columnsSlider->setPageStep(1);
          columnsSlider->setSingleStep(1);
          columnsSlider->setValue(3);
          columnsSlider->setFixedWidth(150);
          QSizePolicy policy = columnsSlider->sizePolicy();
          {
            policy.setHorizontalPolicy(QSizePolicy::Fixed);
            columnsSlider->setSizePolicy(policy);
          }
          imageTopBarLayout->addWidget(columnsSlider);
        }

        groupBar = new QWidget();
        {
          groupBar->setLayout(new QHBoxLayout());
          imageTopBarLayout->addWidget(groupBar, 0, Qt::AlignCenter);

          connect(&lesson, SIGNAL(groupChanged(Group*)), this, SLOT(updateGroupBar()));
          connect(&session, SIGNAL(selectedGroupsChanged(const std::vector<Group*>&)), this, SLOT(updateGroupBar()));
          connect(&session,
                  SIGNAL(selectedAnnotationsChanged(const std::unordered_set<Annotation*>&)),
                  this,
                  SLOT(updateGroupBar()));
        }

        imageCountLabel = new QLabel(tr("Placeholder"));
        {
          imageCountLabel->setAlignment(Qt::AlignCenter);
          imageCountLabel->setFixedWidth(150);
          QSizePolicy policy = imageCountLabel->sizePolicy();
          {
            policy.setHorizontalPolicy(QSizePolicy::Fixed);
            imageCountLabel->setSizePolicy(policy);
          }
          imageTopBarLayout->addWidget(imageCountLabel);
        }

        imageLayout->addWidget(imageTopBar);
      }

      imageScene = new QGraphicsScene(this);
      imageView = new TileGridView(imageScene, this);
      {
        imageView->hide();
        imageView->setColumns(columnsSlider->value());
        imageView->setDragMode(QGraphicsView::RubberBandDrag);
        imageView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        imageView->setSpacing(24.0);
        imageView->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
        imageLayout->addWidget(imageView);

        connect(imageScene, SIGNAL(selectionChanged()), this, SLOT(handleSelections()));
        connect(columnsSlider, SIGNAL(valueChanged(int)), imageView, SLOT(setColumns(int)));
      }

      connect(&lesson, SIGNAL(datasetsRemoved(const std::set<Dataset*>&)), this, SLOT(updateImagePage()));
      connect(&lesson, SIGNAL(groupChanged(Group*)), this, SLOT(updateImagePage()));
      connect(&lesson, SIGNAL(visibleDatasetsChanged(const std::vector<Dataset*>&)), this, SLOT(updateImagePage()));
      connect(&lesson, SIGNAL(visibleGroupsChanged(const std::vector<Group*>&)), this, SLOT(updateImagePage()));
      updateImagePage();

      connect(&session,
              SIGNAL(selectedDataFramesChanged(const std::vector<DataFrame*>&)),
              this,
              SLOT(previewDataFrames(const std::vector<DataFrame*>&)));
      connect(&session,
              SIGNAL(selectedAnnotationsChanged(const std::unordered_set<Annotation*>&)),
              this,
              SLOT(updateSelectedAnnotations(const std::unordered_set<Annotation*>&)));
    }

    QWidget* annotationPage = new QWidget();
    {
      pages->addWidget(annotationPage);

      QVBoxLayout* annotationLayout = new QVBoxLayout();
      annotationPage->setLayout(annotationLayout);

      QLabel* annotationPagePrompt = new QLabel(tr("To be implemented."));
      {
        annotationPagePrompt->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        annotationLayout->addWidget(annotationPagePrompt);
      }
    }

    layout->addWidget(pages);

    connect(modeSwitcher, SIGNAL(buttonClicked(int)), pages, SLOT(setCurrentIndex(int)));
  }

  this->setLayout(layout);
}

QWidget* TeachWidget::getWidget() {
  return this;
}

void TeachWidget::updateGroupBar() {
  std::cout << "updateGroupBar selectedGroups.size: " << session.selectedGroups.size() << std::endl;
  std::cout << "lesson.annotationToGroups.size(): " << lesson.annotationToGroups.size() << std::endl;

  QList<GroupCheckBox*> existingCheckboxes = groupBar->findChildren<GroupCheckBox*>();

  const std::unordered_set<Annotation*>& selected = session.selectedAnnotations;
  const int checkboxesNeeded = session.selectedGroups.size();

  // Remove extra checkboxes (if too many existing ones)
  while (existingCheckboxes.size() > checkboxesNeeded) {
    delete existingCheckboxes.front();
    existingCheckboxes.pop_front();
  }

  // Add more checkboxes if existing ones are not enough
  while (existingCheckboxes.size() < checkboxesNeeded) {
    GroupCheckBox* checkbox = new GroupCheckBox();
    {
      checkbox->setTristate(true);
      QSizePolicy policy = checkbox->sizePolicy();
      {
        policy.setHorizontalPolicy(QSizePolicy::Maximum);
        checkbox->setSizePolicy(policy);
      }
      groupBar->layout()->addWidget(checkbox);

      connect(checkbox, SIGNAL(stateChanged(int)), this, SLOT(sendAnnotationMembership()));
    }
    existingCheckboxes.push_back(checkbox);
  }

  // Count how many annotations are selected from each group
  std::unordered_map<Group*, int> groupSizes;
  for (const auto& group : session.selectedGroups) {
    const auto& members = group->members;

    groupSizes[group] = std::count_if(members.begin(), members.end(), [&selected](Annotation* const& member) {
      return selected.find(member) != selected.end();
    });
  }

  // Update the checkboxes
  checkboxToGroup.clear();
  size_t shortcutIdx = 0;
  for (const auto& group : session.selectedGroups) {
    GroupCheckBox* checkbox = existingCheckboxes.front();
    existingCheckboxes.pop_front();

    // Update mapping to Group
    checkboxToGroup.insert(std::make_pair(checkbox, group));

    // Disable the "ungrouped" checkbox
    // TODO Set font italics too?
    const bool enabled = (group != &(lesson.ungrouped));
    if (checkbox->isEnabled() != enabled) {
      checkbox->setEnabled(enabled);
    }

    // Update keyboard shortcuts
    if (shortcutIdx < checkboxShortcuts.size()) {
      QShortcut* shortcut = checkboxShortcuts[shortcutIdx];
      disconnect(shortcut, SIGNAL(activated()), 0, 0);
      connect(shortcut, SIGNAL(activated()), checkbox, SLOT(click()));
    } else if (shortcutIdx < 9) { // Only add shortcuts for 1 through 9
      QShortcut* shortcut = new QShortcut(Qt::Key_1 + shortcutIdx, this);
      shortcut->setAutoRepeat(false);
      shortcut->setContext(Qt::WidgetWithChildrenShortcut);
      connect(shortcut, SIGNAL(activated()), checkbox, SLOT(click()));
      checkboxShortcuts.push_back(shortcut);
    }
    ++shortcutIdx;

    const int groupSize = groupSizes.at(group);

    Qt::CheckState checkState;
    if (groupSize == 0) {
      checkState = Qt::Unchecked;
    } else if (groupSize == (int)selected.size()) {
      checkState = Qt::Checked;
    } else {
      checkState = Qt::PartiallyChecked;
    }

    if (checkbox->checkState() != checkState) {
      const bool wasBlocked = checkbox->blockSignals(true);
      checkbox->setCheckState(checkState);
      checkbox->blockSignals(wasBlocked);
    }

    // Set checkbox color
    const int colorBoxSize = 20;
    QPixmap pixmap(colorBoxSize, colorBoxSize);
    pixmap.fill(group->color);
    checkbox->setIcon({pixmap});

    // Set checkbox label
    std::string text = group->name + " (" + std::to_string(groupSize) + ")";
    checkbox->setText(text.c_str());
  }
}

void TeachWidget::sendAnnotationMembership() {
  GroupCheckBox* checkbox = qobject_cast<GroupCheckBox*>(sender());
  if (!checkbox) {
    return;
  }

  Group* group = checkboxToGroup.at(checkbox);

  if (checkbox->checkState() == Qt::Checked) {
    lesson.addToGroup(group, session.selectedAnnotations);
  } else {
    lesson.removeFromGroup(group, session.selectedAnnotations);
  }
}

void TeachWidget::updateImagePage() {
  if (lesson.datasets.empty()) {
    imagePagePrompt->setText(tr("Load a dataset or open a Lesson to get started."));
    imagePagePrompt->show();
    imageTopBar->hide();
    imageView->hide();

  } else if (lesson.visibleDatasets.empty() && lesson.visibleGroups.empty()) {
    imagePagePrompt->setText(tr("Select a dataset or a group to view it here."));
    imagePagePrompt->show();
    imageTopBar->hide();
    imageView->hide();

  } else {
    redrawImageView(lesson.combineDatasets(lesson.visibleDatasets), lesson.visibleGroups);
    imagePagePrompt->hide();
    imageTopBar->show();
    imageView->show();
  }
}

void TeachWidget::redrawImageView(const std::vector<DataFrame*>& framesIn, const std::vector<Group*>& groups) {

  // TODO Plan better way to handle custom ordering (of datasets, groups, etc)
  //      Currently always assumes Datasets ordered lexicographically, frames by time

  QTime t;
  t.start();

  // Get frames we want to show in any case
  std::set<DataFrame*> frames(framesIn.begin(), framesIn.end());

  // Get list of visible annotations given list of visible groups & build lookup table for groups this annotation belongs to
  std::unordered_set<Annotation*> annotations;
  std::multimap<Annotation*, Group*> annotationToGroups;
  for (Group* group : groups) {
    for (Annotation* annotation : group->members) {
      annotations.insert(annotation);
      annotationToGroups.insert(std::make_pair(annotation, group));
    }
  }

  // Construct lookup table for annotations on each frame
  std::unordered_multimap<DataFrame*, Annotation*> frameToAnnotations;
  for (Annotation* annotation : annotations) {
    DataFrame* frame = annotation->frame;
    frames.insert(frame);
    frameToAnnotations.insert(std::make_pair(frame, annotation));
  }

  std::cout << "redrawImageView: " << t.elapsed() << " ms to construct lookup tables" << std::endl;
  t.restart();

  imageCountLabel->setText(tr(frames.size() == 1 ? "%1 image" : "%1 images").arg(frames.size()));

  // Remove old annotations that are now gone
  {
    // Handle imageScene selections
    const QSet<QGraphicsItem*> selectedItems = imageScene->selectedItems().toSet();
    bool selectedItemsRemoved = false;

    const int oldSize = annotationToItem.size();
    for (auto i = annotationToItem.begin(); i != annotationToItem.end();) {
      Annotation* annotation = i->first;

      if (annotations.find(annotation) == annotations.end()) {
        AnnotationItem* item = i->second;

        if (!selectedItems.contains(item)) {
          // Removing an unselected item is easy
          imageScene->removeItem(item);

        } else {
          // Removing a selected item:
          // Do not emit signal imageScene->selectionChanged() for every single item
          const bool signalsBlocked = imageScene->blockSignals(true);
          imageScene->removeItem(item);
          imageScene->blockSignals(signalsBlocked);
          selectedItemsRemoved = true;
        }

        delete item;
        i = annotationToItem.erase(i);

      } else {
        ++i;
      }
    }

    if (selectedItemsRemoved) {
      // HACK: Force imageScene to emit a selectionChanged() signal (for the entire removal operation)
      QGraphicsPolygonItem* dummy = new QGraphicsPolygonItem();
      {
        const bool signalsBlocked = imageScene->blockSignals(true);
        imageScene->addItem(dummy);
        dummy->setSelected(true);
        imageScene->blockSignals(signalsBlocked);
      }
      dummy->setSelected(false);
      {
        const bool signalsBlocked = imageScene->blockSignals(true);
        imageScene->removeItem(dummy);
        imageScene->blockSignals(signalsBlocked);
      }
    }

    std::cout << "deleted " << oldSize - annotationToItem.size() << " annotations" << std::endl;
  }

  // Remove old frames that are now gone
  {
    const int oldSize = frameToTile.size();
    for (auto i = frameToTile.begin(); i != frameToTile.end();) {
      DataFrame* frame = i->first;
      if (frames.find(frame) == frames.end()) {
        PixmapTile* tile = i->second;
        imageScene->removeItem(tile);
        i = frameToTile.erase(i);
      } else {
        ++i;
      }
    }
    std::cout << "deleted " << oldSize - frameToTile.size() << " tiles" << std::endl;
  }

  // Sort frames to display
  std::vector<DataFrame*> sortedFrames(frames.begin(), frames.end());
  std::sort(sortedFrames.begin(), sortedFrames.end(), [this](DataFrame* fa, DataFrame* fb) {
    // Returns whether fa comes strictly before fb; orders datasets by lexicographically ascending name, then by ascending
    // time
    const auto ia = lesson.frameToDataset.find(fa);
    const auto ib = lesson.frameToDataset.find(fb);
    const auto ie = lesson.frameToDataset.end();
    if (ia == ie || ib == ie) {
      return fa < fb;
    }

    const auto da = ia->second;
    const auto db = ib->second;
    if (da == db || da->name == db->name) {
      return fa->time < fb->time;
    } else {
      return da->name < db->name;
    }
  });

  std::cout << "redrawImageView: " << t.elapsed() << " ms on housekeeping" << std::endl;
  t.restart();

  // Find out which new frames need to be drawn
  int z = -1;
  int newTiles = 0;
  int newItems = 0;
  for (DataFrame* frame : sortedFrames) {
    ++z;
    PixmapTile* tile = nullptr;

    auto j = frameToTile.find(frame);
    if (j == frameToTile.end()) {
      tile = PixmapTile::fromBHImage(frame->get<Image>());
      tile->setData(Qt::UserRole, QVariant::fromValue(frame));
      imageScene->addItem(tile);
      frameToTile.insert(std::make_pair(frame, tile));
      ++newTiles;
    } else {
      tile = j->second;
    }

    if ((int)tile->zValue() != z) {
      tile->setZValue(z);
    }

    if (groups.empty()) {
      tile->setFlag(QGraphicsItem::ItemIsSelectable, true);

    } else {
      // Can select images only if no annotations visible
      tile->setFlag(QGraphicsItem::ItemIsSelectable, false);

      // Draw new annotations
      auto annotations = frameToAnnotations.equal_range(frame);
      for (auto j = annotations.first; j != annotations.second; ++j) {
        Annotation* annotation = j->second;

        // Get ordered list of QColors for this annotation
        QVector<QColor> colors;
        auto groups = annotationToGroups.equal_range(annotation);
        for (auto k = groups.first; k != groups.second; ++k) {
          Group* group = k->second;
          colors.push_back(group->color);
        }

        // Find existing annotationItem or create new one
        AnnotationItem* annotationItem = nullptr;
        {
          auto k = annotationToItem.find(annotation);
          if (k == annotationToItem.end()) {
            annotationItem = boost::apply_visitor(AnnotationItemFactory{annotation, colors}, annotation->info);
            annotationItem->setData(Qt::UserRole, QVariant::fromValue(annotation));
            annotationItem->setFlags(QGraphicsItem::ItemIsSelectable);
            annotationItem->setParentItem(tile);
            annotationToItem.insert(std::make_pair(annotation, annotationItem));
            ++newItems;
          } else {
            annotationItem = k->second;
          }
        }

        // Set new annotationItem colors
        if (colors != annotationItem->colors()) {
          annotationItem->setColors(colors);
          annotationItem->update();
        }
      }
    }
  }
  std::cout << "added " << newTiles << " new tiles" << std::endl;
  std::cout << "added " << newItems << " new annotations" << std::endl;

  updateSelectedAnnotations(session.selectedAnnotations);

  std::cout << "redrawImageView: " << t.elapsed() << " ms to redraw" << std::endl;
}

void TeachWidget::previewDataFrames(const std::vector<DataFrame*>& frames) {
  if (frames.empty() || !session.annotator) {
    return;
  }

  session.annotator->annotate(frames);
}

void TeachWidget::updateSelectedAnnotations(const std::unordered_set<Annotation*>& annotations) {
  std::cout << "updateSelectedAnnotations " << annotations.size() << std::endl;

  const bool wasBlocked = imageScene->blockSignals(true);
  imageScene->clearSelection();
  for (Annotation* annotation : annotations) {
    auto i = annotationToItem.find(annotation);
    if (i != annotationToItem.end()) {
      AnnotationItem* item = i->second;
      item->setSelected(true);
    }
  }
  imageScene->blockSignals(wasBlocked);
}

void TeachWidget::handleSelections() {
  std::vector<DataFrame*> selectedDataFrames;
  std::unordered_set<Annotation*> selectedAnnotations;

  auto selectedItems = imageScene->selectedItems();
  for (auto& item : selectedItems) {
    DataFrame* frame = item->data(Qt::UserRole).value<DataFrame*>();
    Annotation* annotation = item->data(Qt::UserRole).value<Annotation*>();
    if (frame) {
      selectedDataFrames.push_back(frame);
    } else if (annotation) {
      selectedAnnotations.insert(annotation);
    }
  }

  session.selectDataFrames(selectedDataFrames);
  session.selectAnnotations(selectedAnnotations);
}
