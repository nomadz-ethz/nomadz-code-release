/**
 * @file RandomForests.h
 *
 * Declaration of a class containing the random forests for object classification.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>
#include <sys/stat.h>

#include "Tree.h"
#include "TreeBuilder.h"
#include "TreeReadWrite.h"

/** This class represents a set of random trees.
 * @param I input type
 * @param L label type
 * @param F split function class
 *          must have same I in implementation of bool F::operator()(const I&)
 * @param S statistic function class
 *          must have same L in implementation of S::constructor(const std::vector<L>&, const std::vector<int>&)
 */
template <class I, class L, class F, class S> class RandomForest {
private:
  /** Filesystem-related helper functions */
  static inline bool isFile(const std::string& path) {
    struct stat s;
    return stat(path.c_str(), &s) == 0 && S_ISREG(s.st_mode);
  }
  static inline bool isDir(const std::string& path) {
    struct stat s;
    return stat(path.c_str(), &s) == 0 && S_ISDIR(s.st_mode);
  }

public:
  std::vector<DecisionTree<I, S>> trees;
  std::vector<L> classes;

  /** Default constructor.
   *  Don't use this; prefer one of the factory functions (load, train) instead.
   */
  inline RandomForest<I, L, F, S>() : trees(), classes() {}

  /** Returns whether this object is usable to classify data.
   * @return True if can use this object to classify data.
   */
  operator bool() const { return !trees.empty() && !classes.empty(); }

  bool operator==(const RandomForest<I, L, F, S>& other) const { return trees == other.trees && classes == other.classes; }

  /** Returns how many trees this forest contains.
   * @return Number of trees
   */
  inline int treeCount() const { return trees.size(); }

  /** Factory function to load XML trees.
   * @param classes Vector of classes
   * @param paths   Vector of paths to the XML files
   * @return        A properly initialized RandomForest.
   * @throw         A std::runtime_error with a string description if paths could not be loaded.
   */
  static RandomForest<I, L, F, S> loadXML(const std::vector<std::string>& paths, const std::vector<L>& classes) {
    RandomForest<I, L, F, S> forest;

    if (!paths.size())
      throw std::runtime_error(std::string("RandomForest::loadXML: paths is empty"));

    for (auto path = paths.cbegin(); path != paths.cend(); ++path) {
      if (!isFile(*path)) {
        throw std::runtime_error(std::string("RandomForest: file does not exist at ") + *path);
      }

      DecisionTree<I, S> tree = TreeReadWrite<I, S, F>::readTree(*path);
      forest.trees.push_back(std::move(tree));
    }

    forest.classes = classes;
    return forest;
  }

  /** Factory function to load XML trees; class labels implied to be int (0, 1, 2, ...).
   * @param paths Vector of paths to the XML files
   * @return      A properly initialized RandomForest.
   * @throw       A std::runtime_error with a string description if paths could not be loaded.
   */
  static RandomForest<I, int, F, S> loadXML(const std::vector<std::string>& paths) {
    RandomForest<I, int, F, S> forest = loadXML(paths, std::vector<int>());

    // HACK Infer class labels from the recursively left-most StatisticFunction in the first tree
    //      (assumes left-most leaf node has same number of classes as every other leaf node)
    int numClasses = 0;
    if (forest.trees.size() > 0) {
      TreeNodeBase<I, S>* node = forest.trees[0].root.get();

      while (node->getNodeType() != NODE_LEAF) {
        const InnerTreeNode<I, F, S>* inner = dynamic_cast<InnerTreeNode<I, F, S>*>(node);
        node = inner->left.get();
      }

      const LeafTreeNode<I, S>* leaf = dynamic_cast<LeafTreeNode<I, S>*>(node);
      numClasses = leaf->data.probabilities().size();
    }

    for (int i = 0; i < numClasses; ++i) {
      forest.classes.push_back(i);
    }

    return forest;
  }

  /** Factory function to load .bin trees.
   * @param classes Vector of classes
   * @param paths   Vector of paths to the .bin files
   * @return        A properly initialized RandomForest.
   * @throw         A std::runtime_error with a string description if paths could not be loaded.
   */
  static RandomForest<I, L, F, S> loadBin(const std::vector<std::string>& paths, const std::vector<L>& classes) {
    RandomForest<I, L, F, S> forest;

    if (!paths.size())
      throw std::runtime_error(std::string("RandomForest::loadBin: paths is empty"));

    for (auto path = paths.cbegin(); path != paths.cend(); ++path) {
      if (!isFile(*path)) {
        throw std::runtime_error(std::string("RandomForest: file does not exist at ") + *path);
      }

      DecisionTree<I, S> tree = TreeReadWrite<I, S, F>::readTreeBin(*path);
      forest.trees.push_back(std::move(tree));
    }

    forest.classes = classes;
    return forest;
  }

  /** Factory function to load .bin trees; class labels implied to be int (0, 1, 2, ...).
   * @param paths Vector of paths to the .bin files
   * @return      A properly initialized RandomForest.
   * @throw       A std::runtime_error with a string description if paths could not be loaded.
   */
  static RandomForest<I, int, F, S> loadBin(const std::vector<std::string>& paths) {
    RandomForest<I, int, F, S> forest = loadBin(paths, std::vector<int>());

    // HACK Infer class labels from the recursively left-most StatisticFunction in the first tree
    //      (assumes left-most leaf node has same number of classes as every other leaf node)
    int numClasses = 0;
    if (forest.trees.size() > 0) {
      TreeNodeBase<I, S>* node = forest.trees[0].root.get();

      while (node->getNodeType() != NODE_LEAF) {
        const InnerTreeNode<I, F, S>* inner = dynamic_cast<InnerTreeNode<I, F, S>*>(node);
        node = inner->left.get();
      }

      const LeafTreeNode<I, S>* leaf = dynamic_cast<LeafTreeNode<I, S>*>(node);
      numClasses = leaf->data.probabilities().size();
    }

    for (int i = 0; i < numClasses; ++i) {
      forest.classes.push_back(i);
    }

    return forest;
  }

  /** Run input through random forest and output a probability for each label.
   * @param input Data to classify
   * @return      A mapping from classes to probabilities
   */
  std::map<L, float> predict(const I& input) const {
    std::vector<float> probs(classes.size(), 0.f);

    const size_t numTrees = trees.size();
    for (auto tree = trees.cbegin(); tree != trees.cend(); ++tree) {
      const auto& treeProbs = tree->eval(input).probabilities();

      for (size_t i = 0, n = treeProbs.size(); i < n; ++i) {
        probs[i] += treeProbs[i] / numTrees;
      }
    }

    std::map<L, float> labeledProbs;
    for (size_t i = 0, n = classes.size(); i < n; ++i) {
      labeledProbs[classes[i]] = probs[i];
    }

    return labeledProbs;
  }

  /** Run input through random forest and output label with highest probability.
   * @param input Data to classify
   * @return      The label with the highest probability
   */
  inline L predictLabel(const I& input) const { return mostLikelyLabel(predict(input)); }

  /** Get most likely label out of a mapping from classes to probabilities
   * @param probs Mapping from classes to probabilities
   * @return      The label with the highest probability
   */
  static inline L mostLikelyLabel(const std::map<L, float>& probs) {
    const auto mostLikely =
      std::max_element(probs.cbegin(), probs.cend(), [](const std::pair<L, float>& p1, const std::pair<L, float>& p2) {
        return p1.second < p2.second;
      });
    return mostLikely->first;
  }

  /** Get most likely label out of a vector of probabilities, using the classes stored in this instance
   * @param probs Vector of probabilities
   * @return      The label corresponding to the element with highest probability
   */
  inline L mostLikelyLabel(const std::vector<float>& probs) const {
    const auto mostLikely = std::max_element(probs.cbegin(), probs.cend());
    return classes[std::distance(probs.cbegin(), mostLikely)];
  }

  /** Save the trees (no classes) to a directory
   * @param directory Where to save the xml files to, without trailing slash
   * @param prefix    Every file name will start with this
   * @param suffix    Every file will end with this (name suffix + file extension here)
   * @return          Whether the saving succeeded.
   */
  bool saveXML(std::string directory, std::string prefix = "tree", std::string suffix = ".xml") const {
    if (!isDir(directory))
      if (mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0)
        return false;

    const std::string pathPrefix = directory + "/" + prefix;

    for (size_t i = 0, n = trees.size(); i < n; ++i) {
      const auto& tree = trees[i];
      const std::string path = pathPrefix + std::to_string(i) + suffix;
      TreeReadWrite<I, S, F>::writeTree(tree, path);
    }

    return true;
  }

  /** Save the trees (no classes) to a directory
   * @param directory Where to save the bin files to, without trailing slash
   * @param prefix    Every file name will start with this
   * @param suffix    Every file will end with this (name suffix + file extension here)
   * @return          Whether the saving succeeded.
   */
  bool saveBin(std::string directory, std::string prefix = "tree", std::string suffix = ".bin") const {
    if (!isDir(directory))
      if (mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0)
        return false;

    const std::string pathPrefix = directory + "/" + prefix;

    for (size_t i = 0, n = trees.size(); i < n; ++i) {
      const auto& tree = trees[i];
      const std::string path = pathPrefix + std::to_string(i) + suffix;
      TreeReadWrite<I, S, F>::writeTreeBin(tree, path);
    }

    return true;
  }

  /** Train a random forest. May print debug information to std::cout. // TODO Handle verbosity better
   * @param C      cost function class
   *               must have same L in implementation of float E::operator()(const vector<L>&, const vector<int>&)
   * @param data   vector of input data
   * @param labels vector of labels
   * @param builder
   * @return       A properly initialized RandomForest.
   * @throw        A std::runtime_error with a string description if paths could not be loaded.
   */
  template <typename C>
  static RandomForest<I, L, F, S> train(const std::vector<I>& data,
                                        const std::vector<L>& labels,
                                        TreeBuilder<I, F, S, C>& builder,
                                        const int numTrees,
                                        const std::vector<L>& classes) {
    RandomForest<I, L, F, S> forest;
    forest.classes = classes;

    const int N = data.size();

    std::vector<bool> isOob(numTrees * N, true); // bool is-out-of-bag[sample-id * numTrees + tree-id]
    for (int t = 0; t < numTrees; ++t) {
      std::cout << "Training " << (t + 1) << " / " << numTrees << std::endl;
      std::mt19937 engine(std::time(nullptr) * numTrees + t);

      auto startTime = std::chrono::high_resolution_clock::now();

      std::vector<I> subdata;
      std::vector<L> sublabels;

      for (int i = 0; i < N; ++i) {
        const int r = std::uniform_int_distribution<>(0, N - 1)(engine);
        subdata.push_back(data[r]);
        sublabels.push_back(labels[r]);
        isOob[r * numTrees + t] = false;
      }

      forest.trees.push_back(builder.train_tree(subdata, sublabels));

      auto endTime = std::chrono::high_resolution_clock::now();

      // Calculate out-of-bag error; should decrease as more trees are trained.
      std::cout << "Out-of-bag error: " << (1.f - forest.outOfBagError(data, labels, isOob, numTrees)) << std::endl;

      std::cout << "Elapsed: " << ((endTime - startTime).count() / 1e9f) << " s; ";
      const std::string tmpPath = "/tmp/rf-tmp-" + std::to_string(rand() % 10000);
      std::cout << (forest.saveXML(tmpPath) ? "saved: " : "save failed: ") + tmpPath << std::endl;
    }

    return forest;
  }

  /** Train a random forest. May print debug information to std::cout. // TODO Handle verbosity better
   * @param C      cost function class
   *               must have same L in implementation of float E::operator()(const vector<L>&, const vector<int>&)
   * @param data   vector of input data
   * @param labels vector of labels
   * @return       A properly initialized RandomForest.
   * @throw        A std::runtime_error with a string description if paths could not be loaded.
   */
  template <typename C>
  static RandomForest<I, int, F, S>
  train(const std::vector<I>& data, const std::vector<int>& labels, TreeBuilder<I, F, S, C>& builder, const int numTrees) {
    std::set<int> uniqueLabels;
    for (auto i = labels.cbegin(); i != labels.cend(); ++i)
      uniqueLabels.insert(*i);

    std::vector<int> classes;
    for (auto i = uniqueLabels.cbegin(); i != uniqueLabels.cend(); ++i)
      classes.push_back(*i);

    return train<C>(data, labels, builder, numTrees, classes);
  }

  /** Calculate the out-of-bag error of this forest. Usually used during training.
   * @param data     vector of input data
   * @param labels   vector of labels
   * @param isOob    indicates whether a given sample is out-of-bag for a given tree
   *                 used like: bool is-out-of-bag(sample-id, tree-id) := isOob[sample-id * numTrees + tree-id]
   * @param numTrees how many trees isOob has information about
   */
  // TODO Merge isOob & numTrees into same "Bags" class? The whole thing is a lil wonky
  float outOfBagError(const std::vector<I>& data,
                      const std::vector<L>& labels,
                      const std::vector<bool>& isOob,
                      const int numTrees) {
    const int N = data.size();
    const int T = trees.size();
    const int C = classes.size();

    assert((int)isOob.size() >= T * N);

    int correct = 0;
    for (int i = 0; i < N; ++i) {
      const I& sample = data[i];
      const L& label = labels[i];

      std::vector<float> votes(C, 0.f);

      for (int t = 0; t < T; ++t) {
        if (!isOob[i * numTrees + t])
          continue;

        const std::vector<float> treeProbs = trees[t].eval(sample).probabilities();
        for (int j = 0; j < C; ++j)
          votes[j] += treeProbs[j];
      }

      if (mostLikelyLabel(votes) == label)
        ++correct;
    }

    return (float)correct / N;
  }
};
