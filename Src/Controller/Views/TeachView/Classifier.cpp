/**
 * @file Classifier.cpp
 *
 * Implementation of Classifier & ClassifierWorker classes.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <QDir>
#include <opencv2/imgproc/imgproc.hpp>

#include "Classifier.h"
#include "Tools/RandomForest/TreeConverter.h"

Classifier::Classifier(const std::string path, const std::string name) : _path(path), _name(name) {
  if (name.empty() && !path.empty()) {
    // If name not given, set default name to the last part of the given path
    QFileInfo file(path.c_str());
    setName(file.baseName().toUtf8().constData());
  }
}

Classifier* Classifier::createFromPath(const std::string& path) {
  if (OldRandomForestClassifier::canLoad(path)) {
    return new OldRandomForestClassifier(path);

    // In the future, add other classifiers here, like this:
    // if (AmazingClassifier::canLoad(path)) {
    //   return new AmazingClassifier(path);

  } else {
    return nullptr;
  }
}

bool Classifier::load(const std::string& path) {
  if (doLoad(path)) {
    _path = path;
    emit changed(this);
    return true;
  } else {
    return false;
  }
}

std::map<int, float> Classifier::predict(const Sample* sample) {
  if (empty()) {
    load(_path);
  }
  return doPredict(sample);
}

bool Classifier::save(const std::string& path) {
  if (doSave(path)) {
    _path = path;
    emit changed(this);
    return true;
  } else {
    return false;
  }
}

OldRandomForestClassifier::OldRandomForestClassifier(const std::string path, const std::string name)
    : Classifier(path, name), _fileType(FileType::unknown) {
  patchSize = 24;
  maxDepth = 10;
  minSamplesLeaf = 10;
  maxPatchesPerClass = -1;
  maxRandomTests = 500;
  minSamplesSplit = 20;
  numTrees = 20;
}

bool OldRandomForestClassifier::canLoad(const std::string& dir) {
  // Temporary code to convert forest formats
  // if (OldRandomForestClassifier::findFileType(dir) == FileType::xml) {
  //   auto forest = MatRandomForest::loadXML(xmlsInDir(dir));
  //   RandomForest<LightPatch, int, LightPixelDifference, LeafStatistic> forest2 = TreeConverter::convert(forest);
  //   forest2.saveBin(dir + "-bin");
  // }

  return OldRandomForestClassifier::findFileType(dir) != FileType::other;
}

std::string OldRandomForestClassifier::type() const {
  if (!empty()) {
    switch (fileType()) {
    case FileType::bin:
      return std::string("RF (") + std::to_string(forest->treeCount()) + " trees, bin)";
      break;
    case FileType::xml:
      return std::string("RF (") + std::to_string(forest->treeCount()) + " trees, xml)";
      break;
    case FileType::unknown:
      return std::string("RF (") + std::to_string(forest->treeCount()) + " trees)";
      break;
    }
  } else if (isSaved()) {
    switch (fileType()) {
    case FileType::bin:
      return std::string("RF (") + std::to_string(binsInDir(path()).size()) + " trees, bin)";
      break;
    case FileType::xml:
      return std::string("RF (") + std::to_string(xmlsInDir(path()).size()) + " trees, xml)";
      break;
    }
  }

  using std::to_string;
  const auto ft = fileType();
  return std::string("RF ? (empty: ") + to_string(empty()) + ", isSaved: " + to_string(isSaved()) +
         ", bin: " + to_string(ft == FileType::bin) + ", xml: " + to_string(ft == FileType::xml) +
         ", other: " + to_string(ft == FileType::other) + ", unknown: " + to_string(ft == FileType::unknown) + ")";
}

bool OldRandomForestClassifier::doLoad(const std::string& dir) {
  std::string err;

  try {
    switch (findFileType(dir)) {
    case FileType::bin: {
      const auto bins = binsInDir(dir);
      forest.reset(new MatRandomForest(MatRandomForest::loadBin(bins)));
      return true;
    }
    case FileType::xml: {
      const auto xmls = xmlsInDir(dir);
      forest.reset(new MatRandomForest(MatRandomForest::loadXML(xmls)));
      return true;
    }
    default:
      return false;
    }
  } catch (const std::runtime_error& e) {
    std::cerr << "OldRandomForestClassifier::doLoad: couldn't load forest at " << dir << ": " << e.what() << std::endl;
    return false;
  } catch (const cv::Exception& e) {
    std::cerr << "OldRandomForestClassifier::doLoad: couldn't load forest at " << dir << ": (" << e.code << ") " << e.err
              << " in " << e.func << " in " << e.file << ":" << e.line << std::endl;
    return false;
  }

  return false;
}

std::map<int, float> OldRandomForestClassifier::doPredict(const Sample* sample) const {
  const MatSample* matSample = dynamic_cast<const MatSample*>(sample);
  if (forest && matSample) {
    return forest->predict(matSample->mat);
  } else {
    return {};
  }
}

bool OldRandomForestClassifier::doSave(const std::string& dir) const {
  return forest->saveBin(dir) && forest->saveXML(dir);
}

OldRandomForestClassifier::FileType OldRandomForestClassifier::findFileType(const std::string& dir) {
  try {
    const auto bins = binsInDir(dir);
    if (!bins.empty()) {
      MatRandomForest f = MatRandomForest::loadBin(bins);
      return FileType::bin;
    }
  } catch (const std::runtime_error& e) {
    std::cerr << "OldRandomForestClassifier::findFileType: " << dir << " is not of FileType::bin: " << e.what() << std::endl;
  }

  // No bins; try to load xmls
  try {
    const auto xmls = xmlsInDir(dir);
    if (!xmls.empty()) {
      MatRandomForest f = MatRandomForest::loadXML(xmls);
      return FileType::xml;
    }
  } catch (const std::runtime_error& e) {
    std::cerr << "OldRandomForestClassifier::findFileType: " << dir << " is not of FileType::xml: " << e.what() << std::endl;
  } catch (const cv::Exception& e) {
    std::cerr << "OldRandomForestClassifier::findFileType: " << dir << " is not of FileType::xml: (" << e.code << ") "
              << e.err << " in " << e.func << " in " << e.file << ":" << e.line << std::endl;
  }

  std::cerr << "OldRandomForestClassifier::findFileType: is other: " << dir << std::endl;
  return FileType::other;
}

OldRandomForestClassifier::FileType OldRandomForestClassifier::fileType() const {
  if (!isSaved()) {
    return FileType::unknown;
  }

  if (_fileType == FileType::unknown) {
    _fileType = findFileType(_path);
  }

  return _fileType;
}

void OldRandomForestClassifier::train(const std::map<int, std::vector<Sample*>>& inputs) {
  PixelDifference prototypeSplit(patchSize);

  TreeBuilder<cv::Mat, PixelDifference, LeafStatistic, EntropyFunction> builder(
    prototypeSplit, inputs.size(), maxDepth, maxRandomTests, minSamplesSplit, minSamplesLeaf);

  // Prepare inputs for the training
  std::vector<cv::Mat> patches;
  std::vector<int> labels;
  for (const auto& i : inputs) {
    int label = i.first;
    const std::vector<Sample*>& samples = i.second;

    for (Sample* sample : samples) {
      MatSample* matSample = dynamic_cast<MatSample*>(sample);
      patches.push_back(matSample->mat);
      labels.push_back(label);
    }
  }

  // We want a pointer to a forest (stored on the heap, not locally inside this function)
  std::cout << "OldRandomForestClassifier::train: calling MatRandomForest::train" << std::endl;
  forest.reset(new MatRandomForest(MatRandomForest::train<EntropyFunction>(patches, labels, builder, numTrees)));
  std::cout << "OldRandomForestClassifier::train: done calling MatRandomForest::train" << std::endl;
}

std::vector<std::string> OldRandomForestClassifier::binsInDir(const std::string& dir) {
  // Get vector of files inside dir
  QDir qDir(dir.c_str());
  QStringList filters;
  filters << "*.bin";
  const auto qPaths = qDir.entryList(filters, QDir::Files, QDir::Name);

  // Keep the bin files & get full absolute path
  std::vector<std::string> paths;
  for (const QString& path : qPaths) {
    paths.push_back(dir + "/" + path.toUtf8().constData());
  }

  return paths;
}

std::vector<std::string> OldRandomForestClassifier::xmlsInDir(const std::string& dir) {
  // Get vector of files inside dir
  QDir qDir(dir.c_str());
  QStringList filters;
  filters << "*.xml";
  const auto qPaths = qDir.entryList(filters, QDir::Files, QDir::Name);

  // Keep the xml files & get full absolute path
  std::vector<std::string> paths;
  for (const QString& path : qPaths) {
    paths.push_back(dir + "/" + path.toUtf8().constData());
  }

  return paths;
}

void MeanClassificationWorker::setParameters() {
  // TODO Store parameters inside class here
  return;
}

void MeanClassificationWorker::start() {
  std::cout << "MeanClassificationWorker::start" << std::endl;
  if (!results.empty()) {
    std::cerr << "MeanClassificationWorker: cannot start: previous results not yet consumed" << std::endl;
    return;
  }

  int processed = 0;
  for (const auto& i : inputs) {
    ++processed;

    Annotation* annotation = i.first;
    const std::vector<Sample*>& samples = i.second;

    std::map<int, float> probs; // unnormalized probabilities (sum of probs of each sample)
    for (Sample* sample : samples) {
      const std::map<int, float> sampleProbs = classifier.predict(sample);
      for (const auto& i : sampleProbs) {
        int label = i.first;
        float prob = i.second;

        if (probs.find(label) == probs.end()) {
          probs[label] = prob;
        } else {
          probs[label] += prob;
        }
      }
    }

    results.insert(std::make_pair(annotation, mostLikelyLabel(probs)));

    emit progressed(processed, inputs.size());
  }

  emit finished();
}

std::unordered_map<Annotation*, int> MeanClassificationWorker::giveResults() {
  std::cout << "MeanClassificationWorker::giveResults" << std::endl;

  // Make new empty vector
  std::unordered_map<Annotation*, int> results2;

  // Put this->results into the empty vector results2, while emptying this->results
  std::swap(results, results2);

  // Return the actual results that are now in the no-longer-empty vector
  return results2;
}
