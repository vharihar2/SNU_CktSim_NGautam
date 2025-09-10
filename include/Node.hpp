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
 *
 * @brief Contains the definition of the Node class
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
 *
 * @brief Represents a node in the graph
 *
 * Each node represents a circuit element that is connected between two Node.
 * */

class Node
{
   public:
    std::string name; /**< Name of the node */
    std::vector<std::shared_ptr<Edge>>
        edges;      /**< List of edges connected to the node */
    bool processed; /**< Flag value to know whether it is processed */

    /**
     * @brief		Traverses the map (graph of the circuit) and
     *				populates the MNA and RHS matrices
     *
     * @param	[indexMap] map<string, int>
     *
     * @param	[out] mna The left hand side matrix for the modified
     *nodal analysis equation
     *
     * @param [out] rhs An 1 x n vector representing  the
     *independent voltage sources
     */
    void traverse(std::map<std::string, int> &indexMap,
                  std::vector<std::vector<double>> &mna,
                  std::vector<double> &rhs);
};
