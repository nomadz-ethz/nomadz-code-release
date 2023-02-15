/**
 * @file TeachSession.cpp
 *
 * Implementation of class TeachSession
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <QDir>
#include <QFileInfo>
#include "TeachSession.h"

TeachSession::TeachSession(const Lesson& lesson) : lesson(lesson), annotator(nullptr) {
  connect(
    &lesson, SIGNAL(datasetsRemoved(const std::set<Dataset*>&)), this, SLOT(removeDatasets(const std::set<Dataset*>&)));
  connect(&lesson, SIGNAL(groupsRemoved(const std::set<Group*>&)), this, SLOT(removeGroups(const std::set<Group*>&)));
}

void TeachSession::selectDatasets(const std::vector<Dataset*>& newSelection) {
  if (selectedDatasets != newSelection) {
    selectedDatasets = newSelection;
    emit selectedDatasetsChanged(selectedDatasets);
  }
}

void TeachSession::storeAnnotator(Annotator* newAnnotator) {
  if (annotator.get() != newAnnotator) {
    annotator.reset(newAnnotator); // If there was another annotator before, destroy it
    emit annotatorChanged();
  }
}

void TeachSession::selectDataFrames(const std::vector<DataFrame*>& newSelection) {
  if (selectedDataFrames != newSelection) {
    selectedDataFrames = newSelection;
    emit selectedDataFramesChanged(selectedDataFrames);
  }
}

void TeachSession::selectAnnotations(const std::unordered_set<Annotation*>& annotations) {
  if (selectedAnnotations != annotations) {
    selectedAnnotations = annotations;
    emit selectedAnnotationsChanged(selectedAnnotations);
  }
}

void TeachSession::selectGroups(const std::vector<Group*>& newSelection) {
  if (selectedGroups != newSelection) {
    selectedGroups = newSelection;
    emit selectedGroupsChanged(selectedGroups);
  }
}

void TeachSession::removeDatasets(const std::set<Dataset*>& removedDatasets) {
  std::set<DataFrame*> removedFrames;
  for (Dataset* dataset : removedDatasets) {
    for (const auto& i : dataset->frames) {
      DataFrame* frame = i.second.get();
      removedFrames.insert(frame);
    }
  }

  // Remove from selectedDataFrames
  if (removeFromVector(selectedDataFrames, removedFrames) != 0) {
    emit selectedDataFramesChanged(selectedDataFrames);
  }

  // Remove from selectedDatasets
  if (removeFromVector(selectedDatasets, removedDatasets) != 0) {
    emit selectedDatasetsChanged(selectedDatasets);
  }
}

void TeachSession::removeGroups(const std::set<Group*>& removedGroups) {
  // Remove from selectedGroups
  if (removeFromVector(selectedGroups, removedGroups) != 0) {
    emit selectedGroupsChanged(selectedGroups);
  }
}

void TeachSession::storeClassifier(std::unique_ptr<Classifier> newClassifier) {
  std::vector<std::unique_ptr<Classifier>> tmp;
  tmp.push_back(std::move(newClassifier));
  storeClassifiers(std::move(tmp));
}

void TeachSession::storeClassifiers(std::vector<std::unique_ptr<Classifier>> newClassifiers) {
  if (!newClassifiers.empty()) {
    std::vector<Classifier*> added;

    for (auto& newClassifier : newClassifiers) {
      added.push_back(newClassifier.get());
      classifiers.insert(std::make_pair(newClassifier.get(), std::move(newClassifier)));
    }

    emit classifiersAdded(added);
  }
}

void TeachSession::deleteClassifiers(const std::set<Classifier*>& removedClassifiers) {
  // Remove from classifiers
  std::vector<std::unique_ptr<Classifier>> removed;

  for (Classifier* removedClassifier : removedClassifiers) {
    auto i = classifiers.find(removedClassifier);
    if (i != classifiers.end()) {
      std::unique_ptr<Classifier>& classifier = i->second;
      removed.push_back(std::move(classifier));
      classifiers.erase(i);
    }
  }

  if (!removed.empty()) {
    std::set<Classifier*> removed2;

    std::transform(removed.begin(),
                   removed.end(),
                   std::inserter(removed2, removed2.begin()),
                   [](const std::unique_ptr<Classifier>& c) -> Classifier* { return c.get(); });

    emit classifiersRemoved(removed2);
  }
}

void TeachSession::updateClassifiersInDir(const std::string& dirIn, const std::vector<std::string>& pathsIn) {
  QDir dir(dirIn.c_str());

  // Put new paths in a sorted set for faster lookup
  std::set<std::string> paths(pathsIn.begin(), pathsIn.end());

  // Flag for deletion those classifiers whose paths are deleted from the "dir" directory
  // and remove from "paths" those that already have a Classifier in TeachSession
  std::set<Classifier*> toDelete;
  for (const auto& i : classifiers) {
    Classifier* classifier = i.first;
    const std::string& path = classifier->path();
    if (path.empty()) {
      continue;
    }

    QFileInfo file(path.c_str());

    // Don't do anything to classifiers outside of "dir"
    if (file.dir().absolutePath() == dir.absolutePath()) {
      auto j = paths.find(path);
      if (j == paths.end()) {
        toDelete.insert(classifier);
      } else {
        paths.erase(j);
      }
    }
  }

  if (!toDelete.empty()) {
    deleteClassifiers(toDelete);
  }

  // Create new Classifiers for new classifiers from the filesystem
  std::vector<std::unique_ptr<Classifier>> toStore;
  for (const std::string& path : paths) {
    QFileInfo file(path.c_str());

    std::unique_ptr<Classifier> classifier(Classifier::createFromPath(path));
    if (classifier) {
      toStore.push_back(std::move(classifier));
    }
  }

  if (!toStore.empty()) {
    storeClassifiers(std::move(toStore));
  }
}
