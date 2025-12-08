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
 * @file Edge.hpp
 * @brief Graph edge representing a connection between two circuit nodes.
 *
 * This header declares the `Edge` type used by the circuit graph utilities.
 * An `Edge` connects two `Node` instances and associates that connection with
 * a `CircuitElement` (for example a resistor, source, or reactive element).
 *
 * The structure is intentionally simple: ownership is expressed via
 * `std::shared_ptr` so graph traversal and lifetime management are
 * straightforward for the solver and analysis code.
 */

#pragma once

#include <memory>

// Forward declarations
class Node;
class CircuitElement;

/**
 * @class Edge
 * @brief Represents a connection between two nodes in the circuit graph.
 *
 * An `Edge` models the electrical connection between `source` and `target`
 * nodes and stores a pointer to the corresponding `CircuitElement`. The graph
 * utilities use `Edge` instances to build adjacency lists and traverse the
 * circuit topology.
 */
class Edge
{
   public:
    /**
     * @brief Shared pointer to the source node (where the element's positive
     *        terminal is connected).
     *
     * May be null if the node was removed or not yet resolved.
     */
    std::shared_ptr<Node> source;

    /**
     * @brief Shared pointer to the target node (where the element's negative
     *        terminal is connected).
     *
     * May be null if the node was removed or not yet resolved.
     */
    std::shared_ptr<Node> target;

    /**
     * @brief Shared pointer to the circuit element associated with this edge.
     *
     * The element drives the electrical behaviour of the connection (R, C, L,
     * source, controlled source, etc.).
     */
    std::shared_ptr<CircuitElement> circuitElement;
};
