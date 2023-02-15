/**
 * @file Classifier.h
 *
 * Declaration of Classifier & ClassificationWorker classes.
 * ClassificationWorkers use Classifiers to turn every set of Samples into an integer (a label).
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <memory>
#include <vector>
#include <QMetaType>
#include <QObject>
#include <opencv2/core/core.hpp>

#include "Controller/Views/TeachView/Sampler.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/RandomForest/CostFunctions.h"
#include "Tools/RandomForest/RandomForest.h"
#include "Tools/RandomForest/SplitFunctions.h"
#include "Tools/RandomForest/StatisticFunctions.h"
#include "Tools/RandomForest/TreeBuilder.h"

/**
 * @class Classifier
 *
 * Can be initialized with an actual classifier inside, or just a file path.
 *
 * Can be:
 *  - empty (can't do anything)
 *  - only on disk
 *  - only in memory
 *  - both on disk and in memory
 */
class Classifier : public QObject {
public:
  Classifier(const std::string path = "", const std::string name = "");

  static Classifier* createFromPath(const std::string&);

  inline bool isSaved() const { return !_path.empty(); }

  bool load(const std::string&);

  inline std::string name() const { return _name; }

  inline std::string path() const { return _path; }

  // If not loaded, attempts to load from disk
  std::map<int, float> predict(const Sample*);

  bool save(const std::string&);

  void setName(const std::string& name) {
    if (_name != name) {
      _name = name;
      emit changed(this);
    }
  }

  virtual bool empty() const = 0;

  virtual void train(const std::map<int, std::vector<Sample*>>&) = 0;

  virtual std::string type() const { return "Unknown"; }

signals:
  void changed(Classifier*); // emitted when name, path, or loaded/saved status changes
  void finished();
  void progressed(int, int);

private:
  Q_OBJECT

protected:
  std::string _name;

  // If this classifier exists on disk, this is its path
  std::string _path;

  virtual bool doLoad(const std::string&) = 0;
  virtual std::map<int, float> doPredict(const Sample*) const = 0;
  virtual bool doSave(const std::string&) const = 0;
};

Q_DECLARE_METATYPE(Classifier*);

class OldRandomForestClassifier : public Classifier {
public:
  enum class FileType {
    unknown, // used when the concept of "file type" doesn't make sense, e.g. when !isSaved()
    other,
    bin,
    xml
  };

  typedef RandomForest<cv::Mat, int, PixelDifference, LeafStatistic> MatRandomForest;
  typedef RandomForest<LightPatch, int, LightPixelDifference, LeafStatistic> LightRandomForest;

  OldRandomForestClassifier(const std::string path = "", const std::string name = "");

  static bool canLoad(const std::string&);

  bool doLoad(const std::string&) override;

  std::map<int, float> doPredict(const Sample*) const override;

  bool doSave(const std::string&) const override;

  FileType fileType() const;

  static FileType findFileType(const std::string&);

  bool empty() const override { return !forest; }

  void train(const std::map<int, std::vector<Sample*>>&) override;

  std::string type() const override;

  std::unique_ptr<MatRandomForest> forest;

private:
  int patchSize;
  int maxDepth;
  int minSamplesLeaf;
  int maxPatchesPerClass;
  int maxRandomTests;
  int minSamplesSplit;
  int numTrees;

  mutable FileType _fileType;

  // Returns list of absolute paths to bin files inside a directory
  static std::vector<std::string> binsInDir(const std::string&);

  // Returns list of absolute paths to xml files inside a directory
  static std::vector<std::string> xmlsInDir(const std::string&);
};

class ClassificationWorker : public QObject {
public:
  ClassificationWorker(Classifier& classifier, const std::unordered_map<Annotation*, std::vector<Sample*>>& samples)
      : classifier(classifier), inputs(samples) {}
  virtual ~ClassificationWorker(){};

  virtual std::unordered_map<Annotation*, int> giveResults() = 0;

public slots:
  virtual void start() = 0;

signals:
  void finished();
  void progressed(int, int);

private:
  Q_OBJECT

protected:
  Classifier& classifier;
  std::unordered_map<Annotation*, std::vector<Sample*>> inputs;

  template <typename L> static inline L mostLikelyLabel(const std::map<L, float>& probs) {
    const auto mostLikely =
      std::max_element(probs.cbegin(), probs.cend(), [](const std::pair<L, float>& p1, const std::pair<L, float>& p2) {
        return p1.second < p2.second;
      });
    return mostLikely->first;
  }
};

/**
 * @class MeanClassificationWorker
 *
 * The MeanClassificationWorker turns each set of samples associated with an
 * annotation into an integer label by taking the most likely label from the
 * mean of each sample's label probability.
 */
class MeanClassificationWorker : public ClassificationWorker {
public:
  // MeanClassificationWorker(std::vector<Sample*> samples) : ClassificationWorker(samples) {}
  MeanClassificationWorker(Classifier& classifier, const std::unordered_map<Annotation*, std::vector<Sample*>>& samples)
      : ClassificationWorker(classifier, samples) {}

  std::unordered_map<Annotation*, int> giveResults() override;

  void setParameters();

  void start() override;

private:
  std::unordered_map<Annotation*, int> results;
};
