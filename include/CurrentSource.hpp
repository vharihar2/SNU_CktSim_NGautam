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
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "CircuitElement.hpp"

// Forward declaration of Parser used by the parse factory.
class Parser;

/**
 * @file CurrentSource.hpp
 * @brief Declaration of the CurrentSource element.
 *
 * This header declares the `CurrentSource` class which models an independent
 * current source in the circuit. The class derives from `CircuitElement` and
 * provides the DC stamping behavior as well as a factory `parse` routine used
 * by the netlist parser.
 *
 * DC stamping convention:
 *  - The source injects `value` amps from `nodeA` -> `nodeB`.
 *  - For assembly this results in: rhs[nodeA] -= value, rhs[nodeB] += value.
 *  - Ground node is represented by the name `"0"` and is handled specially
 *    (no RHS entry for ground).
 */
class CurrentSource : public CircuitElement
{
   public:
    /**
     * @brief Construct a new CurrentSource element
     *
     * @param name Element identifier (e.g., "I1").
     * @param nodeA Positive/source node name (current leaves this node).
     * @param nodeB Negative/sink node name (current enters this node).
     * @param value Current in Amperes (non-zero).
     */
    CurrentSource(const std::string& name, const std::string& nodeA,
                  const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }

    /**
     * @brief Stamp the current source into the MNA system (DC).
     *
     * For DC analysis a current source contributes only to the RHS (current
     * injections). The stamping follows the project's sign convention:
     *   - Subtract `value` at `nodeA` (if not ground)
     *   - Add `value` at `nodeB` (if not ground)
     *
     * @param mna Modified Nodal Analysis matrix (unused for ideal current
     *            sources in DC).
     * @param rhs Right-hand side vector (modified in-place).
     * @param indexMap Map from node names to indices in `mna`/`rhs`. The
     * special node name `"0"` denotes ground.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parse a current source from tokenized netlist input.
     *
     * Expected token formats:
     *   Iname nodeA nodeB value
     *
     * On success returns a `std::shared_ptr<CircuitElement>` pointing to the
     * created `CurrentSource`. On failure returns `nullptr` and emits
     * diagnostics via the parser/io layer.
     *
     * @param parser Reference to the Parser helper for validation and parsing.
     * @param tokens Tokenized netlist line for the current source.
     * @param lineNumber Line number in the netlist (for diagnostics).
     * @return std::shared_ptr<CircuitElement> Created current source or nullptr
     * on error.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
};
