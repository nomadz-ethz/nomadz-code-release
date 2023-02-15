/**
 * @file TeachSession.h
 *
 * Declaration of class TeachSession
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <libgen.h>
#include <map>
#include <set>
#include <string>
#include <QObject> // to emit Qt signals
#include <QString>
#include "Controller/Representations/Lesson.h"
#include "Controller/Views/TeachView/Annotator.h"
#include "Controller/Views/TeachView/Classifier.h"
#include "Core/MessageQueue/InMessage.h"

/**
 * @class TeachSession
 *
 * Contains selections in the UI, annotators, and other data that disappears after SimRobot is closed.
 */
class TeachSession : public QObject {
public:
  TeachSession(const Lesson& lesson);

  const Lesson& lesson;

  std::vector<Dataset*> selectedDatasets;
  void selectDatasets(const std::vector<Dataset*>&);

  std::unique_ptr<Annotator> annotator;
  void storeAnnotator(Annotator* annotator);

  std::vector<DataFrame*> selectedDataFrames;
  void selectDataFrames(const std::vector<DataFrame*>&);

  std::unordered_set<Annotation*> selectedAnnotations;
  void selectAnnotations(const std::unordered_set<Annotation*>&);

  std::vector<Group*> selectedGroups;
  void selectGroups(const std::vector<Group*>&);

  std::unordered_map<Classifier*, std::unique_ptr<Classifier>> classifiers;
  void storeClassifier(std::unique_ptr<Classifier>);
  void storeClassifiers(std::vector<std::unique_ptr<Classifier>>);
  void deleteClassifiers(const std::set<Classifier*>&);

  // Synchronize classifiers from a given directory with what's in the actual directory
  // dirIn and pathsIn must be full absolute paths
  // TODO Allow relative pathsIn too (maybe relative to dirIn)?
  void updateClassifiersInDir(const std::string&, const std::vector<std::string>&);

signals:
  void selectedDatasetsChanged(const std::vector<Dataset*>&);

  void annotatorChanged();

  void selectedDataFramesChanged(const std::vector<DataFrame*>&);

  void selectedAnnotationsChanged(const std::unordered_set<Annotation*>&);

  void selectedGroupsChanged(const std::vector<Group*>&);

  void classifiersAdded(const std::vector<Classifier*>&);
  void classifiersRemoved(const std::set<Classifier*>&);

private slots:
  void removeDatasets(const std::set<Dataset*>&);
  void removeGroups(const std::set<Group*>&);

private:
  Q_OBJECT;

  // Remove from the vector those elements that appear both in the vector and in the set
  // Returns number of elements removed from the vector
  template <typename T> int removeFromVector(std::vector<T>& original, const std::set<T>& removed) {
    std::vector<T> result;
    std::set<T> sorted(original.cbegin(), original.cend());

    // result = original - removed
    std::set_difference(sorted.cbegin(), sorted.cend(), removed.cbegin(), removed.cend(), std::back_inserter(result));

    const int count = original.size() - result.size();
    if (count != 0) {
      std::swap(original, result);
    }

    return count;
  }
};
