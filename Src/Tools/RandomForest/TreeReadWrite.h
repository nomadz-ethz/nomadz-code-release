/**
 * @file TreeReadWrite.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <memory>
#include <stdexcept>
#include <opencv2/core/core.hpp>

#include "Tree.h"
#include "Core/Streams/InStreams.h"
#include "Core/Streams/OutStreams.h"

//  Template<I> : Input data
//  Template<L> : Label data
//  Template<F> : split structure overload operator() + constructor randomly construct a new split_function
//  Template<E> : compute entropy of given assignment
//  Template<S> : leaf statistics

template <class I, class S, class F> class TreeReadWrite {

private:
  // Reads FileNode from XML and returns TreeNode
  static std::unique_ptr<TreeNodeBase<I, S>> readLeftAndRight(const cv::FileNode& currentFileNode, int depth) {
    // Check type of tree node
    int type = 0;
    currentFileNode["Type"] >> type;

    if (type == NODE_INNER) {
      // For a inner node, create split_function object and read left and right
      std::unique_ptr<InnerTreeNode<I, F, S>> currentTreeNode(new InnerTreeNode<I, F, S>(depth));
      currentTreeNode->depth = depth;

      currentTreeNode->split_function = F::load(currentFileNode);

      cv::FileNode leftFileNode = currentFileNode["Left"];
      cv::FileNode rightFileNode = currentFileNode["Right"];

      currentTreeNode->left = readLeftAndRight(leftFileNode, depth + 1);
      currentTreeNode->right = readLeftAndRight(rightFileNode, depth + 1);

      return std::move(currentTreeNode);

    } else if (type == NODE_LEAF) {
      // For a leaf node, create statistic object
      std::unique_ptr<LeafTreeNode<I, S>> leafNode(new LeafTreeNode<I, S>(depth + 1, S::load(currentFileNode)));
      return std::move(leafNode);
    } else {
      return nullptr;
    }
  }

  static std::unique_ptr<TreeNodeBase<I, S>> readLeftAndRightBin(InBinaryFile& file, int depth) {
    NodeType type;
    file >> type;

    if (type == NODE_INNER) {
      std::unique_ptr<InnerTreeNode<I, F, S>> currentTreeNode(new InnerTreeNode<I, F, S>(depth));
      currentTreeNode->depth = depth;

      file >> currentTreeNode->split_function;

      bool hasLeft;
      file >> hasLeft;

      if (hasLeft) {
        currentTreeNode->left = readLeftAndRightBin(file, depth + 1);
      }

      bool hasRight;
      file >> hasRight;

      if (hasRight) {
        currentTreeNode->right = readLeftAndRightBin(file, depth + 1);
      }

      return std::move(currentTreeNode);

    } else if (type == NODE_LEAF) {
      std::unique_ptr<LeafTreeNode<I, S>> leafNode(new LeafTreeNode<I, S>(depth + 1));
      file >> leafNode->data;
      return std::move(leafNode);

    } else {
      return nullptr;
    }
  }

  static void writeLeftAndRightBin(const TreeNodeBase<I, S>& currentNode, OutBinaryFile& file) {
    file << currentNode.getNodeType();

    if (currentNode.getNodeType() == NODE_INNER) {
      const InnerTreeNode<I, F, S>& innerNode = dynamic_cast<const InnerTreeNode<I, F, S>&>(currentNode);
      file << innerNode.split_function;

      const bool hasLeft = (innerNode.left != NULL);
      file << hasLeft;

      if (hasLeft) {
        writeLeftAndRightBin(*(innerNode.left), file);
      }

      const bool hasRight = (innerNode.right != NULL);
      file << hasRight;

      if (hasRight) {
        writeLeftAndRightBin(*(innerNode.right), file);
      }

    } else if (currentNode.getNodeType() == NODE_LEAF) {
      const LeafTreeNode<I, S>& leafNode = dynamic_cast<const LeafTreeNode<I, S>&>(currentNode);
      file << leafNode.data;
    }
  }

  // Writes FileNode to XML
  static void writeLeftAndRight(const TreeNodeBase<I, S>& currentNode, cv::FileStorage& fs) {

    if (currentNode.getNodeType() == NODE_INNER) {
      InnerTreeNode<I, F, S>& innerNode = (InnerTreeNode<I, F, S>&)currentNode;
      fs << "Type" << NODE_INNER;
      innerNode.split_function.save(fs);

      if (innerNode.left != NULL) {
        fs << "Left"
           << "{";
        writeLeftAndRight(*(innerNode.left), fs);
        fs << "}";
      }
      if (innerNode.right != NULL) {
        fs << "Right"
           << "{";
        writeLeftAndRight(*(innerNode.right), fs);
        fs << "}";
      }
    } else if (currentNode.getNodeType() == NODE_LEAF) {
      fs << "Type" << NODE_LEAF;
      ((LeafTreeNode<I, S>&)currentNode).data.save(fs);
    }
  }

public:
  static DecisionTree<I, S> readTreeBin(std::string filename) {
    InBinaryFile file(filename);

    char header[2]; // should be 'r' 'f'
    char version;   // should be 1 :)
    file >> header[0] >> header[1] >> version;

    DecisionTree<I, S> tree;
    file >> tree.maxDepth;
    file >> tree.numberRandomTests;
    file >> tree.minSamplesSplit;
    file >> tree.minSamplesLeaf;

    tree.root = std::move(readLeftAndRightBin(file, 0));

    return tree;
  }

  // Read tree from XML; if problem reading file, throws runtime_error.
  static DecisionTree<I, S> readTree(std::string filename) {
    cv::FileStorage fs;
    bool success = fs.open(filename, cv::FileStorage::READ);

    if (!success) {
      throw std::runtime_error(std::string("TreeReadWrite::readTree could not read tree at ") + filename);
    }

    // Read the base node and start to read recursively
    cv::FileNode node = fs.getFirstTopLevelNode();

    DecisionTree<I, S> tree(readLeftAndRight(node["Base"], 0));
    tree.maxDepth = node["MaxDepth"];
    tree.numberRandomTests = node["NumberRandomTestsPerNode"];
    tree.minSamplesSplit = node["MinSamplesSplit"];
    tree.minSamplesLeaf = node["MinSamplesLeaf"];

    fs.release();

    return tree;
  }

  static void writeTreeBin(const DecisionTree<I, S>& tree, std::string filename) {
    OutBinaryFile file(filename);

    const char version = 1;
    file << 'r' << 'f' << version;

    file << tree.maxDepth;
    file << tree.numberRandomTests;
    file << tree.minSamplesSplit;
    file << tree.minSamplesLeaf;

    writeLeftAndRightBin(*(tree.root), file);
  }

  // Write tree to XML
  static void writeTree(const DecisionTree<I, S>& tree, std::string filename) {
    // Create the storage and open a tree node
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "Tree"
       << "{";
    fs << "MaxDepth" << tree.maxDepth;
    fs << "NumberRandomTestsPerNode" << tree.numberRandomTests;
    fs << "MinSamplesSplit" << tree.minSamplesSplit;
    fs << "MinSamplesLeaf" << tree.minSamplesLeaf;

    fs << "Base"
       << "{";

    writeLeftAndRight(*(tree.root), fs);

    fs << "}";
    fs << "}";
    fs.release();
  }
};
