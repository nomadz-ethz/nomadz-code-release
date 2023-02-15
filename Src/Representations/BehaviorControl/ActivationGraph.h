/**
 * @file ActivationGraph
 *
 * The graph of executed options and states.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Max Risler
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/activation_graph.hpp"
#endif
STREAMABLE_DECLARE(ActivationGraph)
#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/activation_graph_node.hpp"
#endif
STREAMABLE_DECLARE(ActivationGraphNode)

STREAMABLE_ROS(ActivationGraph, {
public:
  STREAMABLE_ROS(ActivationGraphNode, {
    public : inline ActivationGraphNode(
      const std::string& option, int depth, const std::string& state, int optionTime, int stateTime),
    FIELD_WRAPPER_DEFAULT(std::string, nomadz_msgs::msg::ActivationGraphNode::option, option),
    FIELD_WRAPPER(int, 0, nomadz_msgs::msg::ActivationGraphNode::depth, depth),
    FIELD_WRAPPER_DEFAULT(std::string, nomadz_msgs::msg::ActivationGraphNode::state, state),
    FIELD_WRAPPER(int, 0, nomadz_msgs::msg::ActivationGraphNode::option_time, optionTime),
    FIELD_WRAPPER(int, 0, nomadz_msgs::msg::ActivationGraphNode::state_time, stateTime),
  })
  , FIELD_WRAPPER_DEFAULT(std::vector<ActivationGraphNode>, nomadz_msgs::msg::ActivationGraph::graph, graph),

    // Initialization
    graph.reserve(100);
});

ActivationGraph::ActivationGraphNode::ActivationGraphNode(
  const std::string& option, int depth, const std::string& state, int optionTime, int stateTime)
    : ActivationGraphNode() {
  this->option = option;
  this->depth = depth;
  this->state = state;
  this->optionTime = optionTime;
  this->stateTime = stateTime;
}
