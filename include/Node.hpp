/*
 * Copyright (c) 2022, Shiv Nadar University, Delhi NCR, India. All Rights
 * Reserved. Permission to use, copy, modify and distribute this software for
 * educational, research, and not-for-profit purposes, without fee and without a
 * signed license agreement, is hereby granted, provided that this paragraph and
 * the following two paragraphs appear in all copies, modifications, and
 * distributions.
 *
 * IN NO EVENT SHALL SHIV NADAR UNIVERSITY BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST
 * PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE.
 *
 * SHIV NADAR UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS PROVIDED "AS IS". SHIV
 * NADAR UNIVERSITY HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 */

/**
 * @file Node.hpp
 * @brief Defines the Node class for representing nodes in a circuit graph.
 *
 * This file contains the definition of the Node class, which models a node in
 * the circuit graph. Each node can be connected to multiple edges and is used
 * in the construction and traversal of the circuit for analysis.
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Edge.hpp"

// Forward Declaration
class Edge;

/**
 * @class Node
 * @brief Represents a node in the circuit graph.
 *
 * The Node class models a node in the circuit, which can be connected to
 * multiple edges (circuit elements). Nodes are used to build the graph
 * representation of the circuit for analysis and simulation.
 */
class Node
{
   public:
    /**
     * @brief Name of the node (unique identifier)
     */
    std::string name;
    /**
     * @brief List of edges (connections) attached to this node
     */
    std::vector<std::shared_ptr<Edge>> edges;
    /**
     * @brief Flag indicating whether this node has been processed during
     * traversal
     */
    bool processed;

    /**
     * @brief Traverses the circuit graph from this node and populates the MNA
     * and RHS matrices.
     *
     * This function visits all connected edges and updates the Modified Nodal
     * Analysis (MNA) matrix and the right-hand side (RHS) vector for circuit
     * analysis.
     *
     * @param indexMap Map from node/element names to matrix indices
     * @param mna Reference to the MNA matrix to be populated
     * @param rhs Reference to the RHS vector to be populated
     */
    void traverse(std::map<std::string, int> &indexMap,
                  std::vector<std::vector<double>> &mna,
                  std::vector<double> &rhs);
};
