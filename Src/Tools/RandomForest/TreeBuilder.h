/**
 * @file TreeBuilder.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <random>
#include "Tree.h"

//  Template<I> : Input data
//  Template<F> : split structure overload operator() + constructor randomly construct a new split_function
//  Template<C> : compute criterion function for given assignment
//  Template<S> : leaf statistics

template <class I, class F, class S, class C> class TreeBuilder {
public:
  F prototype_f;
  size_t number_classes;
  int max_depth;
  int min_samples_split;
  int min_samples_leaf;
  int nbr_random_test_per_node;
  std::mt19937 mt;

  TreeBuilder(F pf, size_t nc, int md = 5, int rt = 1000, int mss = 20, int msl = 1) {
    prototype_f = pf;
    number_classes = nc;
    max_depth = md;
    nbr_random_test_per_node = rt;
    min_samples_split = mss;
    min_samples_leaf = msl;

    std::random_device r;
    std::seed_seq seed{r(), r(), r(), r(), r(), r(), r(), r()};
    mt = std::mt19937(seed);
  }

  DecisionTree<I, S> train_tree(const std::vector<I>& inputs, const std::vector<int>& labels) {
    assert(inputs.size() == labels.size());

    std::queue<std::reference_wrapper<std::unique_ptr<TreeNodeBase<I, S>>>> parent_to_train;
    std::queue<int> parent_depth;
    std::queue<std::vector<int>> input_to_node;

    InnerTreeNode<I, F, S> parent_root(-1);
    std::vector<int> all_input(inputs.size());
    for (int i = 0; i < inputs.size(); i++)
      all_input[i] = i;

    parent_to_train.push(std::ref(parent_root.right));
    input_to_node.push(all_input);
    parent_depth.push(-1);

    C cost_function;

    //  Create the tree
    while (!parent_to_train.empty()) {
      float best_cost = INFINITY;
      std::vector<bool> directions;
      F best_split_function;

      // For all inner node look at how we can extend it
      std::reference_wrapper<std::unique_ptr<TreeNodeBase<I, S>>> parent_ptr = parent_to_train.front();
      parent_to_train.pop();
      std::vector<int> current_input = input_to_node.front();
      input_to_node.pop();
      std::vector<bool> association(current_input.size());
      int p_depth = parent_depth.front();
      parent_depth.pop();

      // Randomly try some splitting functions
      std::vector<int> left_input, right_input;
      if (current_input.size() > size_t(min_samples_split)) {
        for (int i = 0; i < nbr_random_test_per_node; i++) {
          F new_split_function(prototype_f);
          new_split_function.randomize(mt);
          new_split_function.optimize(inputs, labels, current_input);

          // Split inputs into (left_input, right_input) according to new function
          std::vector<int> left_input, right_input;
          for (int j = 0; j < association.size(); j++) {
            association[j] = new_split_function(inputs[current_input[j]]);
            if (!association[j])
              left_input.push_back(current_input[j]);
            else
              right_input.push_back(current_input[j]);
          }

          const float ratio = ((float)right_input.size()) / current_input.size();
          const float cost = ratio * cost_function(labels, right_input, number_classes) +
                             (1 - ratio) * cost_function(labels, left_input, number_classes);

          if (cost < best_cost) {
            best_cost = cost;
            best_split_function = new_split_function;
            directions = association;
          }
        }

        for (int j = 0; j < current_input.size(); j++) {
          if (!directions[j])
            left_input.push_back(current_input[j]);
          else
            right_input.push_back(current_input[j]);
        }
      }

      //  Did not manage to further split the data --> creating a leaf
      if (p_depth == max_depth - 1 || left_input.size() < min_samples_leaf || right_input.size() < min_samples_leaf ||
          current_input.size() <= min_samples_split) {
        std::unique_ptr<LeafTreeNode<I, S>> leaf(
          new LeafTreeNode<I, S>(p_depth + 1, S::create(labels, current_input, number_classes)));
        parent_ptr.get() = std::move(leaf);

      } else {
        std::unique_ptr<InnerTreeNode<I, F, S>> node(new InnerTreeNode<I, F, S>(p_depth + 1));
        node->split_function = best_split_function;

        parent_to_train.push(std::ref(node->left));
        input_to_node.push(left_input);
        parent_depth.push(p_depth + 1);

        parent_to_train.push(std::ref(node->right));
        input_to_node.push(right_input);
        parent_depth.push(p_depth + 1);

        parent_ptr.get() = std::move(node);
      }
    }

    DecisionTree<I, S> tree(std::move(parent_root.right));
    tree.maxDepth = max_depth;
    tree.numberRandomTests = nbr_random_test_per_node;
    tree.minSamplesSplit = min_samples_split;
    tree.minSamplesLeaf = min_samples_leaf;
    return tree;
  }
};
