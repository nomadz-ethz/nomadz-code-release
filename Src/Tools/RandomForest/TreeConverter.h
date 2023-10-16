/**
 * @file TreeConverter.h
 *
 * Converts between DecisionTrees of various types
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <opencv2/core/core.hpp>

#include "LightPatch.h"
#include "RandomForest.h"
#include "SplitFunctions.h"
#include "StatisticFunctions.h"
#include "Tree.h"

//  Template<I> : Input data
//  Template<L> : Label data
//  Template<F> : split structure overload operator() + constructor randomly construct a new split_function
//  Template<E> : compute entropy of given assignment
//  Template<S> : leaf statistics

class TreeConverter {
public:
  // Converts RandomForest from <cv::Mat, int, PixelDifference, LeafStatistic> to <LightPatch, int, LightPixelDifference,
  // LeafStatistic>
  static RandomForest<LightPatch, int, LightPixelDifference, LeafStatistic>
  convert(const RandomForest<cv::Mat, int, PixelDifference, LeafStatistic>& in) {
    RandomForest<LightPatch, int, LightPixelDifference, LeafStatistic> out;
    for (const auto& tree : in.trees) {
      out.trees.push_back(convert(tree));
    }
    return out;
  }

  // Converts RandomForest from <LightPatch, int, LightPixelDifference, LeafStatistic> to <cv::Mat, int, PixelDifference,
  // LeafStatistic>
  static RandomForest<cv::Mat, int, PixelDifference, LeafStatistic>
  convert(const RandomForest<LightPatch, int, LightPixelDifference, LeafStatistic>& in, int patchSize) {
    RandomForest<cv::Mat, int, PixelDifference, LeafStatistic> out;
    for (const auto& tree : in.trees) {
      out.trees.push_back(convert(tree, patchSize));
    }
    return out;
  }

  // Converts DecisionTree from <cv::Mat, PixelDifference, LeafStatistic> to <LightPatch, LightPixelDifference,
  // LeafStatistic>
  static DecisionTree<LightPatch, LeafStatistic> convert(const DecisionTree<cv::Mat, LeafStatistic>& in) {
    const int patchSize = maxSplitCoord<cv::Mat, PixelDifference, LeafStatistic, int>(*(in.root));
    return DecisionTree<LightPatch, LeafStatistic>(
      convertNode<cv::Mat, PixelDifference, LeafStatistic, LightPatch, LightPixelDifference, LeafStatistic>(
        *(in.root), 0, patchSize));
  }

  // Converts DecisionTree from <LightPatch, LightPixelDifference, LeafStatistic> to <cv::Mat, PixelDifference,
  // LeafStatistic>
  static DecisionTree<cv::Mat, LeafStatistic> convert(const DecisionTree<LightPatch, LeafStatistic>& in, int patchSize) {
    return DecisionTree<cv::Mat, LeafStatistic>(
      convertNode<LightPatch, LightPixelDifference, LeafStatistic, cv::Mat, PixelDifference, LeafStatistic>(
        *(in.root), 0, patchSize));
  }

private:
  template <class I, class F, class S, typename Coord = int> static Coord maxSplitCoord(const TreeNodeBase<I, S>& node) {
    Coord n = 0;

    if (node.getNodeType() == NODE_INNER) {
      const auto& innerNode = dynamic_cast<const InnerTreeNode<I, F, S>&>(node);
      F f = innerNode.split_function;
      n = std::max<Coord>(n, f.pixel1.x);
      n = std::max<Coord>(n, f.pixel1.y);
      n = std::max<Coord>(n, f.pixel2.x);
      n = std::max<Coord>(n, f.pixel2.y);

      if (innerNode.left) {
        n = std::max<Coord>(n, maxSplitCoord<I, F, S, Coord>(*(innerNode.left)));
      }

      if (innerNode.right) {
        n = std::max<Coord>(n, maxSplitCoord<I, F, S, Coord>(*(innerNode.right)));
      }
    }

    return n;
  }

  // static std::unique_ptr<TreeNodeBase<LightPatch, LeafStatistic>> convertNode(const TreeNodeBase<cv::Mat, LeafStatistic>&
  // in, int depth, int patchSize) {
  //   if (in.getNodeType() == NODE_INNER) {
  //     const auto& innerNode = dynamic_cast<const InnerTreeNode<cv::Mat, PixelDifference, LeafStatistic>&>(in);

  //     std::unique_ptr<InnerTreeNode<LightPatch, LightPixelDifference, LeafStatistic>> out(
  //       new InnerTreeNode<LightPatch, LightPixelDifference, LeafStatistic>(depth)
  //     );

  //     out->depth = depth;
  //     out->split_function = LightPixelDifference(innerNode.split_function, patchSize);

  //     if (innerNode.left) {
  //       out->left = convertNode(*(innerNode.left), depth + 1, patchSize);
  //     }

  //     if (innerNode.right) {
  //       out->right = convertNode(*(innerNode.right), depth + 1, patchSize);
  //     }

  //     return std::move(out);

  //   } else if (in.getNodeType() == NODE_LEAF) {
  //     const auto& leafNode = dynamic_cast<const LeafTreeNode<cv::Mat, LeafStatistic>&>(in);

  //     std::unique_ptr<LeafTreeNode<LightPatch, LeafStatistic>> out(
  //       new LeafTreeNode<LightPatch, LeafStatistic>(depth, leafNode.data)
  //     );

  //     return std::move(out);
  //   }
  // }

  // Convert a TreeNodeBase<I1, S1> (with split function F1) to a new TreeNodeBase<I2, S2> (with split function F2)
  // Assumes converting constructor F2(const F1 other&) exists
  template <class I1, class F1, class S1, class I2, class F2, class S2>
  static std::unique_ptr<TreeNodeBase<I2, S2>> convertNode(const TreeNodeBase<I1, S1>& in, int depth, int patchSize) {
    if (in.getNodeType() == NODE_INNER) {
      const auto& innerNode = dynamic_cast<const InnerTreeNode<I1, F1, S1>&>(in);

      std::unique_ptr<InnerTreeNode<I2, F2, S2>> out(new InnerTreeNode<I2, F2, S2>(depth));

      out->depth = depth;
      out->split_function = F2(innerNode.split_function, patchSize);

      if (innerNode.left)
        out->left = convertNode<I1, F1, S1, I2, F2, S2>(*(innerNode.left), depth + 1, patchSize);
      if (innerNode.right)
        out->right = convertNode<I1, F1, S1, I2, F2, S2>(*(innerNode.right), depth + 1, patchSize);

      return std::move(out);

    } else if (in.getNodeType() == NODE_LEAF) {
      const auto& leafNode = dynamic_cast<const LeafTreeNode<I1, S1>&>(in);

      std::unique_ptr<LeafTreeNode<I2, S2>> out(new LeafTreeNode<I2, S2>(depth, leafNode.data));

      return std::move(out);
    }
    return nullptr;
  }
};
