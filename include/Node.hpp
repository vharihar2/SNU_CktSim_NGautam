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
 * @brief Node representation used by the circuit connectivity graph.
 *
 * This header declares the `Node` class used to build the circuit topology
 * graph. Nodes hold adjacency lists of `Edge` objects (each referencing a
 * `CircuitElement`) and provide a traversal routine used during MNA assembly.
 *
 * The Node API is intentionally minimal: the traversal logic calls element
 * stamping routines and performs a depth-first walk of the connectivity graph.
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

// Forward declaration to keep header lightweight.
class Edge;

/**
 * @class Node
 * @brief Graph node representing an electrical node in the circuit.
 *
 * A `Node` connects to zero or more `Edge` instances. Each edge references a
 * circuit element and the adjacent node. The `traverse` method visits the
 * connected subgraph and invokes stamping routines on elements to populate the
 * MNA matrix and RHS vector.
 */
class Node
{
   public:
    /** @brief Node name (unique identifier, e.g., \"0\" for ground). */
    std::string name;

    /** @brief Adjacency list of edges attached to this node. */
    std::vector<std::shared_ptr<Edge>> edges;

    /** @brief Traversal flag; true when the node has been visited. */
    bool processed = false;

    /**
     * @brief Traverse the connectivity graph starting from this node.
     *
     * This routine performs a depth-first traversal over adjacent nodes,
     * invoking `stamp` or `stampTransient` on each unprocessed element as it
     * encounters edges. Use `isTransient` to select transient stamping paths.
     *
     * The function updates the provided MNA matrix and RHS vector in-place.
     *
     * @param indexMap Mapping from node/element names to indices in
     * `mna`/`rhs`.
     * @param mna Modified Nodal Analysis matrix to populate (mutated in-place).
     * @param rhs Right-hand side vector to populate (mutated in-place).
     * @param isTransient If true, use transient stamping (`stampTransient`);
     * otherwise use DC `stamp`.
     */
    void traverse(std::map<std::string, int> &indexMap,
                  std::vector<std::vector<double>> &mna,
                  std::vector<double> &rhs, bool isTransient = false);
};
