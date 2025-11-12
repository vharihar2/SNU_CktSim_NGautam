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
 * @brief Defines the Edge class for representing connections between nodes in a
 * circuit graph.
 *
 * This file contains the definition of the Edge class, which models a
 * connection (edge) between two nodes in the circuit graph. Each edge is
 * associated with a circuit element.
 */

#pragma once

#include <memory>

// Forward Declaration
class Node;
class CircuitElement;

/**
 * @class Edge
 * @brief Represents a connection (edge) between two nodes in the circuit graph.
 *
 * The Edge class models a connection between two nodes in the circuit. Each
 * edge is associated with a specific circuit element (such as a resistor,
 * capacitor, etc.).
 */
class Edge
{
   public:
    /**
     * @brief Pointer to the source (starting) node of the edge
     */
    std::shared_ptr<Node> source;
    /**
     * @brief Pointer to the target (ending) node of the edge
     */
    std::shared_ptr<Node> target;
    /**
     * @brief Pointer to the circuit element that this edge represents
     */
    std::shared_ptr<CircuitElement> circuitElement;
};
