/**
 * @file Lesson.cpp
 *
 * Implementation of class Lesson
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include "Lesson.h"
#include "Representations/Infrastructure/Image.h"

Lesson::Lesson() : ungrouped("ungrouped", QColor(255, 255, 255, 255)) {
  connect(&ungrouped, SIGNAL(changed(Group*)), this, SLOT(reemitGroupChanged(Group*)));
}

void Lesson::clear() {
  {
    std::set<Group*> removedGroups;
    for (const auto& i : groups) {
      removedGroups.insert(i.first);
    }

    if (!visibleGroups.empty()) {
      visibleGroups.clear();
      emit visibleGroupsChanged(visibleGroups);
    }

    ungrouped.clearMembers();
    annotationToGroups.clear();

    // Clear this->groups, but keep its unique_ptrs alive just for one more emit
    decltype(groups) blackHole;
    std::swap(groups, blackHole);

    emit groupsRemoved(removedGroups);
  }

  {
    std::set<Annotation*> removedAnnotations;
    for (const auto& i : annotations) {
      removedAnnotations.insert(i.first);
    }

    // Clear this->annotations, but keep its unique_ptrs alive just for one more emit
    decltype(annotations) blackHole;
    std::swap(annotations, blackHole);

    // emit annotationsRemoved(removedAnnotations);
  }

  {
    std::set<Dataset*> removedDatasets;
    for (const auto& i : datasets) {
      removedDatasets.insert(i.first);
    }

    if (!visibleDatasets.empty()) {
      visibleDatasets.clear();
      emit visibleDatasetsChanged(visibleDatasets);
    }

    frameToDataset.clear();

    // Clear this->datasets, but keep its unique_ptrs alive just for one more emit
    decltype(datasets) blackHole;
    std::swap(datasets, blackHole);

    emit datasetsRemoved(removedDatasets);
  }
}

void Lesson::merge(const StreamableLesson& streamableLesson) {
  // Construct lookup table for (DataFrame index -> StreamableDataFrame*)
  // (to be used when loading Datasets later)
  std::unordered_map<int, const StreamableDataFrame*> frameIdToStreamable;
  for (const auto& i : streamableLesson.frames) {
    frameIdToStreamable.insert(std::make_pair(i.id, &i));
  }

  // Prepare to store datasets & frames
  std::unordered_map<int, Dataset*> idToDataset;
  std::vector<std::unique_ptr<Dataset>> newDatasets;
  std::unordered_map<int, DataFrame*> idToFrame;
  idToDataset.reserve(streamableLesson.datasets.size());
  newDatasets.reserve(streamableLesson.datasets.size());
  idToFrame.reserve(streamableLesson.frames.size());

  // Load datasets & frames from their sources
  for (const auto& i : streamableLesson.datasets) {
    std::unique_ptr<Dataset> dataset = Dataset::load(i, frameIdToStreamable, idToFrame);
    if (dataset) {
      idToDataset.insert(std::make_pair(i.id, dataset.get()));
      newDatasets.push_back(std::move(dataset));
    } else {
      throw DatasetLoadError(std::string("Lesson::merge: Failed to load StreamableDataset with name: ") + i.name +
                               ", sourcePath: " + i.sourcePath,
                             i);
    }
  }
  storeDatasets(std::move(newDatasets));

  // Show some of the new datasets
  std::vector<Dataset*> newVisibleDatasets(visibleDatasets);
  for (const auto& i : streamableLesson.visibleDatasets) {
    if (idToDataset.find(i) != idToDataset.end()) {
      newVisibleDatasets.push_back(idToDataset.at(i));
    } else {
      std::cerr << "Lesson::merge: Failed to find dataset id " << i << " when adding visible datasets" << std::endl;
    }
  }
  showDatasets(newVisibleDatasets);

  // Prepare to store annotations
  std::unordered_map<int, Annotation*> idToAnnotation;
  std::vector<std::unique_ptr<Annotation>> newAnnotations;
  idToAnnotation.reserve(streamableLesson.circleAnnotations.size() + streamableLesson.polygonAnnotations.size());
  newAnnotations.reserve(streamableLesson.circleAnnotations.size() + streamableLesson.polygonAnnotations.size());

  // Convert & store circle annotations
  for (const auto& i : streamableLesson.circleAnnotations) {
    std::unique_ptr<Annotation> annotation(new Annotation(i, idToFrame));
    idToAnnotation.insert(std::make_pair(i.id, annotation.get()));
    newAnnotations.push_back(std::move(annotation));
  }

  // Convert & store polygon annotations
  for (const auto& i : streamableLesson.polygonAnnotations) {
    std::unique_ptr<Annotation> annotation(new Annotation(i, idToFrame));
    idToAnnotation.insert(std::make_pair(i.id, annotation.get()));
    newAnnotations.push_back(std::move(annotation));
  }
  storeAnnotations(std::move(newAnnotations));

  // Convert & store Groups
  std::unordered_map<int, Group*> idToGroup;
  std::vector<std::unique_ptr<Group>> newGroups;
  idToGroup.reserve(streamableLesson.groups.size());
  newGroups.reserve(streamableLesson.groups.size());
  for (const auto& i : streamableLesson.groups) {
    std::unique_ptr<Group> group(new Group(i, idToAnnotation));
    idToGroup.insert(std::make_pair(i.id, group.get()));
    newGroups.push_back(std::move(group));
  }
  storeGroups(std::move(newGroups));

  // Show some of the new groups
  std::vector<Group*> newVisibleGroups(visibleGroups);
  for (const auto& i : streamableLesson.visibleGroups) {
    if (idToGroup.find(i) != idToGroup.end()) {
      newVisibleGroups.push_back(idToGroup.at(i));
    } else {
      std::cerr << "Lesson::merge: Failed to find group id " << i << " when adding visible groups" << std::endl;
    }
  }

  if (streamableLesson.ungroupedVisible &&
      std::find(newVisibleGroups.begin(), newVisibleGroups.end(), &ungrouped) == newVisibleGroups.end()) {
    newVisibleGroups.push_back(&ungrouped);
  }

  showGroups(newVisibleGroups);
}

StreamableLesson::StreamableLesson(const Lesson& lesson) {
  // Convert & store frames, and construct a lookup table for (DataFrame* -> DataFrame index)
  std::unordered_map<DataFrame*, int> frameToId;
  for (const auto& i : lesson.datasets) {
    Dataset* dataset = i.first;
    for (const auto& j : dataset->frames) {
      DataFrame* frame = j.second.get();
      int frameId = frames.size();

      frames.push_back(StreamableDataFrame(frameId, *frame));
      frameToId.insert(std::make_pair(frame, frameId));
    }
  }

  // Convert & store datasets, and construct a lookup table for (Dataset* -> Dataset index)
  std::unordered_map<Dataset*, int> datasetToId;
  for (const auto& i : lesson.datasets) {
    Dataset* dataset = i.first;
    int datasetId = datasets.size();

    datasets.push_back(StreamableDataset(datasetId, *dataset, frameToId));
    datasetToId.insert(std::make_pair(dataset, datasetId));
  }

  // Store ids of visible datasets
  for (const auto& dataset : lesson.visibleDatasets) {
    visibleDatasets.push_back(datasetToId.at(dataset));
  }

  // Convert & store annotations, and construct a lookup table for (Annotation* -> Annotation index)
  std::unordered_map<Annotation*, int> annotationToId;
  for (const auto& i : lesson.annotations) {
    Annotation* annotation = i.first;
    int annotationId = annotationToId.size();

    annotationToId.insert(std::make_pair(annotation, annotationId));
    if (annotation->info.type() == typeid(Annotation::Circle)) {
      circleAnnotations.push_back(StreamableCircleAnnotation(annotationId, *annotation, frameToId));
    } else if (annotation->info.type() == typeid(Annotation::Polygon)) {
      polygonAnnotations.push_back(StreamablePolygonAnnotation(annotationId, *annotation, frameToId));
    }
  }

  // Convert & store groups, and construct a lookup table for (Group* -> Group index)
  std::unordered_map<Group*, int> groupToId;
  for (const auto& i : lesson.groups) {
    Group* group = i.first;
    int groupId = groups.size();

    groups.push_back(StreamableGroup(groupId, *group, annotationToId));
    groupToId.insert(std::make_pair(group, groupId));
  }

  // Store ids of visible groups
  ungroupedVisible = false;
  for (const auto& group : lesson.visibleGroups) {
    if (group == &(lesson.ungrouped)) {
      ungroupedVisible = true;
    } else {
      visibleGroups.push_back(groupToId.at(group));
    }
  }
}

void Lesson::storeDataset(std::unique_ptr<Dataset> newDataset) {
  std::vector<std::unique_ptr<Dataset>> tmp;
  tmp.push_back(std::move(newDataset));
  storeDatasets(std::move(tmp));
}

void Lesson::storeDatasets(std::vector<std::unique_ptr<Dataset>> newDatasets) {
  std::vector<Dataset*> added;

  for (auto& newDataset : newDatasets) {
    auto sameSource = [&newDataset](decltype(*datasets.cbegin()) i) {
      return i.first->source == newDataset->source && i.first->sourcePath == newDataset->sourcePath;
    };

    if (std::none_of(datasets.cbegin(), datasets.cend(), sameSource)) {
      // Update lookup table frameToDataset
      for (auto& i : newDataset->frames) {
        DataFrame* newFrame = i.second.get();
        frameToDataset.insert(std::make_pair(newFrame, newDataset.get()));
      }

      // Add dataset to this Lesson
      added.push_back(newDataset.get());
      datasets.insert(std::make_pair(newDataset.get(), std::move(newDataset)));
    }
  }

  emit datasetsAdded(added);
}

std::vector<DataFrame*> Lesson::combineDatasets(const std::vector<Dataset*>& selected) const {
  std::vector<DataFrame*> frames;
  for (auto i = selected.begin(); i != selected.end(); ++i) {
    const Dataset& dataset = *(datasets.at(*i));
    for (auto j = dataset.frames.begin(); j != dataset.frames.end(); ++j) {
      DataFrame& frame = *j->second;
      if (!frame.has<Image>()) {
        continue;
      }

      frames.push_back(&frame);
    }
  }

  return frames;
}

void Lesson::showDatasets(const std::vector<Dataset*>& datasets) {
  if (visibleDatasets != datasets) {
    visibleDatasets = datasets;
    emit visibleDatasetsChanged(visibleDatasets);
  }
}

void Lesson::storeAnnotations(std::vector<std::unique_ptr<Annotation>> newAnnotations) {
  std::vector<Annotation*> newUngrouped;

  annotations.reserve(annotations.size() + newAnnotations.size());
  for (std::unique_ptr<Annotation>& i : newAnnotations) {
    Annotation* newAnnotation = i.get();
    annotations.insert(std::make_pair(newAnnotation, std::move(i)));
    newUngrouped.push_back(newAnnotation);
  }

  ungrouped.addMembers(newUngrouped);
}

void Lesson::storeAnnotationsAndGroup(std::vector<std::unique_ptr<Annotation>> newAnnotationsIn,
                                      std::unique_ptr<Group> newGroup,
                                      bool show) {
  std::vector<std::unique_ptr<Annotation>> newAnnotations; // These will get stored in Lesson (unique new annotations only)
  std::vector<Annotation*> newGroupMembers; // These will go into newGroup (pointers to a mix of new & existing annotations)

  // Check if any of the annotations already exist & replace pointer
  for (auto& i : newAnnotationsIn) {
    const Annotation& annotation = *(i.get());

    auto same = [&annotation](decltype(*annotations.cbegin()) i) { return *(i.first) == annotation; };

    auto j = std::find_if(annotations.cbegin(), annotations.cend(), same);
    if (j == annotations.cend()) {
      // annotation is new
      newGroupMembers.push_back(i.get());
      annotationToGroups.insert(std::make_pair(i.get(), newGroup.get()));
      newAnnotations.push_back(std::move(i));
    } else {
      // annotation already exists
      Annotation* existingAnnotation = j->first;
      annotationToGroups.insert(std::make_pair(existingAnnotation, newGroup.get()));
      newGroupMembers.push_back(existingAnnotation);
    }
  }

  newGroup->addMembers(newGroupMembers);

  Group* newGroupPtr = newGroup.get();

  storeAnnotations(std::move(newAnnotations));
  storeGroup(std::move(newGroup));

  if (show) {
    std::vector<Group*> newVisibleGroups(visibleGroups);
    newVisibleGroups.push_back(newGroupPtr);
    showGroups(newVisibleGroups);
  }
}

void Lesson::storeGroup(std::unique_ptr<Group> newGroup) {
  std::vector<std::unique_ptr<Group>> tmp;
  tmp.push_back(std::move(newGroup));
  storeGroups(std::move(tmp));
}

void Lesson::storeGroups(std::vector<std::unique_ptr<Group>> newGroups) {
  std::vector<Group*> added;
  std::unordered_set<Annotation*> annotations;

  for (auto& newGroup : newGroups) {
    if (groups.find(newGroup.get()) == groups.end()) {
      for (Annotation* annotation : newGroup->members) {
        annotations.insert(annotation);
        annotationToGroups.insert(std::make_pair(annotation, newGroup.get()));
      }

      // This gets automatically disconnected when the group dies, so no corresponding disconnect (unless ownership gets
      // transferred out)
      connect(newGroup.get(), SIGNAL(changed(Group*)), this, SLOT(reemitGroupChanged(Group*)));

      // Add group to this Lesson
      added.push_back(newGroup.get());
      groups.insert(std::make_pair(newGroup.get(), std::move(newGroup)));
    }
  }

  ungrouped.removeMembers(annotations);

  emit groupsAdded(added);
}

void Lesson::deleteGroup(Group* group) {
  // Delete from visibleGroups
  {
    auto i = std::find(visibleGroups.begin(), visibleGroups.end(), group);
    if (i != visibleGroups.end()) {
      visibleGroups.erase(i);
      emit visibleGroupsChanged(visibleGroups);
    }
  }

  // Remove annotations from group
  removeFromGroup(group, {group->members.begin(), group->members.end()});

  // Delete from groups
  {
    auto i = groups.find(group);
    if (i != groups.end()) {
      // Erase it from groups, but keep its unique_ptr alive just for one more emit
      std::unique_ptr<Group> removedGroup(std::move(i->second));
      groups.erase(i);

      emit groupsRemoved({removedGroup.get()});
    }
  }
}

void Lesson::showGroups(const std::vector<Group*>& groups) {
  if (visibleGroups != groups) {
    visibleGroups = groups;
    emit visibleGroupsChanged(visibleGroups);
  }
}

void Lesson::addToGroup(Group* group, const std::unordered_set<Annotation*>& annotations) {
  for (Annotation* annotation : annotations) {
    // Insert new pair (annotation -> group) only if it's not already in there
    auto sameGroup = [&group](const std::pair<Annotation*, Group*>& i) {
      Group* group2 = i.second;
      return group == group2;
    };

    auto groupRange = annotationToGroups.equal_range(annotation);
    if (std::none_of(groupRange.first, groupRange.second, sameGroup)) {
      annotationToGroups.insert(std::make_pair(annotation, group));
    }
  }

  group->addMembers(annotations);
  ungrouped.removeMembers(annotations);
}

// TODO Review this function when not sleep-deprived
void Lesson::removeFromGroup(Group* group, const std::unordered_set<Annotation*>& annotations) {
  // Update annotationToGroups & ungrouped
  std::vector<Annotation*> newUngrouped;

  for (Annotation* annotation : annotations) {
    auto groups = annotationToGroups.equal_range(annotation);
    int numGroups = std::distance(groups.first, groups.second);
    std::cout << "Lesson::removeFromGroup numGroups: " << numGroups << std::endl;

    for (auto i = groups.first; i != groups.second;) {
      Group* group2 = i->second;
      if (group == group2) {
        i = annotationToGroups.erase(i);
        --numGroups;
      } else {
        ++i;
      }
    }

    assert(numGroups >= 0);
    if (numGroups == 0) {
      newUngrouped.push_back(annotation);
    }
  }

  ungrouped.addMembers(newUngrouped);
  group->removeMembers(annotations);
}

void Lesson::reemitGroupChanged(Group* group) {
  emit groupChanged(group);
}
