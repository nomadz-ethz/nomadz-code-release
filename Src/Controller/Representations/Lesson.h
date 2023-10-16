/**
 * @file Lesson.h
 *
 * Declaration of classes Lesson and Streamable Lesson
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <libgen.h>
#include <map>
#include <set>
#include <string>
#include <QObject> // to emit Qt signals
#include "Annotation.h"
#include "Dataset.h"
#include "Group.h"
#include "Controller/Views/TeachView/Annotator.h"
#include "Core/Streams/AutoStreamable.h"

// Defined later in this file
class StreamableLesson;

/**
 * @class Lesson
 *
 * Owns everything relevant to one training session, including datasets, annotations, groups, and forests.
 *
 * Usage notes:
 *
 *   If users of Lesson store pointers to objects owned by Lesson, they *MUST* also listen to Qt signals of type
 *   typeDeleted(set<Type*>) from Lesson, which are emitted whenever the objects these pointers point to
 *   have been destroyed, and delete their pointers accordingly.
 *   Otherwise, users will risk holding dangling pointers to objects already destroyed.
 *
 *   All public members variables are *READ-ONLY*, and *SHOULD NOT* be modified. They remain public only because
 *   `lesson.member` looks nicer without parentheses, and I am too lazy to write public getters everywhere.
 *
 * Miscellaneous notes:
 *
 *   Objects owned by Lesson are stored in containers to unique_ptrs to their address on the heap, instead of
 *   being stored by value within the container, because their identity is tied to their pointer (i.e. one
 *   pointer identifies one unique object). If they were stored by value in containers, standard containers
 *   can invalidate pointers to objects within them, which would "corrupt their identity".
 */
class Lesson : public QObject {
public:
  Lesson();

  // Delete everything in this Lesson object, so that it becomes like a default constructed Lesson again.
  // Emits "deleted" signals as appropriate.
  void clear();

  // Merge the StreamableLesson argument into this Lesson object
  // Ignores datasets with invalid sources
  // Throws exceptions if file has invalid IDs (TODO Throw proper exceptions like "invalid file" something?)
  // Emits "added" signals as appropriate.
  void merge(const StreamableLesson&);

  std::unordered_map<Dataset*, std::unique_ptr<Dataset>> datasets;
  void storeDataset(std::unique_ptr<Dataset>);
  void storeDatasets(std::vector<std::unique_ptr<Dataset>>);
  std::vector<DataFrame*> combineDatasets(const std::vector<Dataset*>&) const;

  std::vector<Dataset*> visibleDatasets;
  void showDatasets(const std::vector<Dataset*>&);

  std::unordered_map<Annotation*, std::unique_ptr<Annotation>> annotations;
  void storeAnnotations(std::vector<std::unique_ptr<Annotation>>);
  void
  storeAnnotationsAndGroup(std::vector<std::unique_ptr<Annotation>>, std::unique_ptr<Group> newGroup, bool show = false);

  std::map<Group*, std::unique_ptr<Group>> groups;
  void storeGroup(std::unique_ptr<Group>);
  void storeGroups(std::vector<std::unique_ptr<Group>>);
  void deleteGroup(Group*);

  std::vector<Group*> visibleGroups;
  void showGroups(const std::vector<Group*>&);

  void addToGroup(Group*, const std::unordered_set<Annotation*>&);
  void removeFromGroup(Group*, const std::unordered_set<Annotation*>&);

  /* Dynamically updated */

  // Lookup table for which dataset a frame belongs to
  std::unordered_map<DataFrame*, Dataset*> frameToDataset;

  // Lookup table for which groups an annotation belongs to (not including the "ungrouped" group)
  std::unordered_multimap<Annotation*, Group*> annotationToGroups;

  // Contains annotations that don't have a group; not saved to StreamableLesson
  Group ungrouped;

signals:
  void datasetsAdded(const std::vector<Dataset*>&);
  void datasetsRemoved(const std::set<Dataset*>&);
  void visibleDatasetsChanged(const std::vector<Dataset*>&);
  void groupsAdded(const std::vector<Group*>&);
  void groupsRemoved(const std::set<Group*>&);
  void groupChanged(Group*);
  void visibleGroupsChanged(const std::vector<Group*>&);

private slots:
  void reemitGroupChanged(Group*); // TODO Read Qt docs and find better way to handle this

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

/**
 * @class StreamableLesson
 *
 * Holds the same information as a Lesson.
 *
 * Can be used as an intermediate data structure to serialize/deserialize a Lesson using B-Human's
 * streaming tools. This allows loading from and saving to files, or transmitting across processes
 * or over a network.
 *
 * DO NOT modify any of the data inside; use a Lesson if you want a safely-mutable object.
 */
STREAMABLE(StreamableLesson, {
  public :
    // Convert from a Lesson
    StreamableLesson(const Lesson&),

  (std::vector<StreamableDataFrame>)frames,

  (std::vector<StreamableDataset>)datasets,
  (std::vector<int>)visibleDatasets,

  // A bit ugly... it would be nice if B-Human's streaming tools can deal with variant classes.
  (std::vector<StreamableCircleAnnotation>)circleAnnotations,
  (std::vector<StreamablePolygonAnnotation>)polygonAnnotations,

  (std::vector<StreamableGroup>)groups,
  (std::vector<int>)visibleGroups,
  (bool)ungroupedVisible,
});

/**
 * @class DatasetLoadError
 *
 * Represents an error while loading a Dataset.
 *
 * Includes the path that could not be loaded.
 */
class DatasetLoadError : public std::runtime_error {
public:
  const StreamableDataset& dataset;

  DatasetLoadError(const std::string& what_arg, const StreamableDataset& dataset)
      : std::runtime_error(what_arg), dataset(dataset) {}
};