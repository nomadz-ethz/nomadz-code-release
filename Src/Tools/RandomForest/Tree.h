/**
 * Tree.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <memory>
#include "Core/Enum.h"

ENUM(NodeType, NODE_INNER, NODE_LEAF);

//  Template<I> : Input data
//  Template<L> : Label data
//  Template<F> : split structure overload operator() + constructor randomly construct a new split_function
//  Template<E> : compute entropy of given assignment

template <class I, class S> class TreeNodeBase {
public:
  virtual ~TreeNodeBase() {}

  int depth;

  TreeNodeBase(int d = -1) : depth(d) {}

  virtual NodeType getNodeType() const = 0;

  virtual const S& eval(const I& i) const = 0;

  virtual bool equals(const TreeNodeBase<I, S>&) const = 0;

  bool operator==(const TreeNodeBase<I, S>& a) const {
    if (typeid(*this) != typeid(a))
      return false;
    return this->equals(a);
  }
};

// Wrapper around the root of a tree, with extra metadata
template <class I, class S> class DecisionTree {
public:
  std::shared_ptr<TreeNodeBase<I, S>> root;

  // Training metadata (not checked in equality comparison)
  int maxDepth;
  int numberRandomTests;
  int minSamplesSplit;
  int minSamplesLeaf;

  DecisionTree() {}

  DecisionTree(std::unique_ptr<TreeNodeBase<I, S>> root) : root(std::move(root)) {}

  const S& eval(const I& i) const { return root->eval(i); }

  bool operator==(const DecisionTree<I, S>& other) const { return *root == *(other.root); }
};

//  template F is the split function of signature bool operator()(const I& i)
template <class I, class F, class S> class InnerTreeNode : public TreeNodeBase<I, S> {

public:
  F split_function;

  std::unique_ptr<TreeNodeBase<I, S>> left;
  std::unique_ptr<TreeNodeBase<I, S>> right;

  InnerTreeNode(int d = -1) : TreeNodeBase<I, S>(d) {
    left = nullptr;
    right = nullptr;
  }

  virtual NodeType getNodeType() const { return NODE_INNER; }

  virtual const S& eval(const I& i) const {
    if (split_function(i))
      return right->eval(i);
    else
      return left->eval(i);
  }

  bool equals(const TreeNodeBase<I, S>& base) const {
    const InnerTreeNode<I, F, S>& other = dynamic_cast<const InnerTreeNode<I, F, S>&>(base);
    return split_function == other.split_function && *left == *(other.left) && *right == *(other.right);
  }
};

template <class I, class S> class LeafTreeNode : public TreeNodeBase<I, S> {
public:
  S data;
  const static int type = NODE_LEAF;

  LeafTreeNode(int d) : TreeNodeBase<I, S>(d) {}

  LeafTreeNode(int d, const S& s) : TreeNodeBase<I, S>(d), data(s) {}

  virtual NodeType getNodeType() const { return NODE_LEAF; }

  virtual const S& eval(const I& i) const { return data; }

  bool equals(const TreeNodeBase<I, S>& base) const {
    const LeafTreeNode<I, S>& other = dynamic_cast<const LeafTreeNode<I, S>&>(base);
    return data == other.data;
  }
};
