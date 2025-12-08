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
 * @file Node.cpp
 * @brief Implementation of Node traversal used to stamp elements into MNA.
 *
 * This file implements `Node::traverse`, which walks the circuit connectivity
 * graph and invokes element stamping routines (DC or transient) to populate
 * the global MNA matrix and RHS vector. The traversal is depth-first and uses
 * the `processed` flag to avoid revisiting nodes/elements.
 */

#include "Node.hpp"

#include "CircuitElement.hpp"
#include "Edge.hpp"

void Node::traverse(std::map<std::string, int> &indexMap,
                    std::vector<std::vector<double>> &mna,
                    std::vector<double> &rhs, bool isTransient)
{
    // Skip ground and already visited nodes.
    if (name == "0" || processed) return;
    processed = true;

    // Stamp each incident element once.
    for (const auto &edge : edges) {
        // Skip elements already stamped by another node traversal.
        if (edge->circuitElement->isProcessed()) continue;
        edge->circuitElement->setProcessed(true);

        // Choose transient or DC stamping according to caller.
        if (isTransient) {
            edge->circuitElement->stampTransient(mna, rhs, indexMap);
        } else {
            edge->circuitElement->stamp(mna, rhs, indexMap);
        }
    }

    // Recurse to adjacent nodes (depth-first).
    for (const auto &edge : edges) {
        if (edge->target) {
            edge->target->traverse(indexMap, mna, rhs, isTransient);
        }
    }
}
