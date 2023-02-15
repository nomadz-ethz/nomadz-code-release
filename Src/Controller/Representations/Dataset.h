/**
 * @file Dataset.h
 *
 * Declaration of class Dataset
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <vector>
#include <QMetaType>
#include <boost/any.hpp>

#include "Core/Enum.h"
#include "Core/MessageQueue/InMessage.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @class DataFrame
 *
 * Owns all of the data that belong to the same frame.
 */
class DataFrame {
public:
  DataFrame(int time = -1) : time(time) {}

  int time;

  /**
   * Returns whether this DataFrame has data of all the requested types.
   * Example usage:
   *   DataFrame f;
   *   bool hasRequirements = f.has<CameraInfo, CameraMatrix>();
   */
  template <typename T> bool has() const { return data.find(std::type_index(typeid(T))) != data.end(); }

  template <typename T1, typename T2, typename... Ts> // Ts can be empty
  bool has() const {
    return has<T1>() && has<T2, Ts...>();
  }

  /**
   * Returns a pointer to an object of the given type.
   * The caller *does not* get ownership of the object.
   * Example usage:
   *   DataFrame f;
   *   Object* o = f.get<Object>();
   */
  template <typename T> const T* get() const {
    return has<T>() ? boost::any_cast<T>(&(data.at(std::type_index(typeid(T))))) : nullptr;
  }

  template <typename T> T* get() { return has<T>() ? boost::any_cast<T>(&(data.at(std::type_index(typeid(T))))) : nullptr; }

  /**
   * Stores a copy of an object. To move an object in, use ::store(std::move(t)).
   * Example usage:
   *   DataFrame f;
   *   Object o;
   *   f.store(o);  // store a copy of o
   * or:
   *   f.store(std::move(o)); // cannot use the variable o after this point
   */
  template <typename T> void store(T t) { data.insert({std::type_index(typeid(T)), t}); }

private:
  std::unordered_map<std::type_index, boost::any> data;
};

Q_DECLARE_METATYPE(DataFrame*);

class StreamableDataFrame;
class StreamableDataset;

/**
 * @class Dataset
 *
 * Owns a time sequence of data imported from one source (logs, folders, etc).
 */
class Dataset {
public:
  ENUM(Source, Log, Folder, Robot);

  // Load a new Dataset from a StreamableDataset, reloading each DataFrame from the source
  // Returns a unique_ptr that owns a Dataset
  //      or a unique_ptr that owns nothing if the source is a Robot or could not be found
  static std::unique_ptr<Dataset> load(const StreamableDataset&,
                                       const std::unordered_map<int, const StreamableDataFrame*>&,
                                       std::unordered_map<int, DataFrame*>&);

  // Save the Dataset as files in a folder
  void saveToFolder(const std::string&) const;

  // Load a new Dataset from the log file in the argument
  // Optionally, specify a vector of frame numbers to load (if empty, every frame gets loaded)
  // Returns a unique_ptr that owns a Dataset
  //      or a unique_ptr that owns nothing if the path was not a valid log file
  static std::unique_ptr<Dataset> loadLog(const std::string&, const std::set<int>& = std::set<int>());

  // Load a new Dataset from the folder in the argument
  // Optionally, specify a vector of frame numbers to load (if empty, every frame gets loaded)
  // Returns a unique_ptr that owns a Dataset
  //      or a unique_ptr that owns nothing if the path was not a valid folder
  static std::unique_ptr<Dataset> loadFolder(const std::string&, const std::set<int>& = std::set<int>());

  // User-facing name
  std::string name;

  // Where this dataset came from
  Source source;

  // Log files: the full absolute path to this log file
  // Folders: the full absolute path to this folder
  // Robot: IP address of robot
  std::string sourcePath;

  // Map of frame numbers to DataFrames
  std::map<int, std::unique_ptr<DataFrame>> frames;

private:
  class LogHandler : public MessageHandler {
  public:
    LogHandler(Dataset& dataset, const std::set<int>& selectedTimes) : dataset(dataset), selectedTimes(selectedTimes) {}

  private:
    Dataset& dataset;
    const std::set<int>& selectedTimes;

    std::unique_ptr<DataFrame> currentFrame = nullptr;

    bool handleMessage(InMessage& message);
  };
};

Q_DECLARE_METATYPE(Dataset*);

/**
 * @class StreamableDataFrame
 *
 * Streamable, stripped down version of a DataFrame object.
 *
 * Does not store the full data, only the frame number.
 */
STREAMABLE(StreamableDataFrame, {
  public : StreamableDataFrame(int id, const DataFrame& frame),

  (int)id,
  (int)time,
});

/**
 * @class StreamableDataset
 *
 * Streamable, stripped down version of a Dataset object.
 *
 * Does not store full data frames, only the frame numbers.
 */
STREAMABLE(StreamableDataset, {
  public : StreamableDataset(int id, const Dataset& dataset, const std::unordered_map<DataFrame*, int>&),

  (int)id,
  (std::string)name,
  (Dataset, Source)source,
  (std::string)sourcePath,
  (std::vector<int>)frameIds,
});
