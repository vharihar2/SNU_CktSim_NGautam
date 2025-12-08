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

// Forward declarations to keep header lightweight
class Parser;

/**
 * @file VoltageSource.hpp
 * @brief Declaration of the voltage source element and its parsing API.
 *
 * The `VoltageSource` class models an ideal independent voltage source for use
 * with the simulator's Modified Nodal Analysis (MNA) framework. Voltage sources
 * are always treated as group-2 elements (their currents are additional
 * unknowns in the system) and therefore require an extra row/column in the
 * MNA matrix. This header documents the public API: construction, stamping and
 * parsing helpers used by the `Parser`.
 *
 * Implementation notes:
 *  - Full API documentation is provided in this header. The corresponding
 *    implementation file `VoltageSource.cpp` should contain a short `@file`
 *    block and focused inline comments documenting implementation details only.
 *  - The static `parse()` helper is used by the parser to construct instances
 *    from tokenized, uppercased netlist lines. It validates token counts,
 *    node names and numeric values via `Parser` helper functions and returns
 *    `nullptr` on parse errors.
 */

/**
 * @class VoltageSource
 * @brief Concrete element representing an independent voltage source.
 *
 * Voltage sources impose a constant voltage difference between two nodes and
 * therefore introduce an extra unknown representing the source current in the
 * MNA system. The `stamp()` method inserts the required couplings between the
 * node-voltage unknowns and the source-current unknown and writes the source's
 * contribution to the RHS vector.
 *
 * Typical netlist forms accepted by the parser:
 *   - Vname nodeA nodeB value
 *
 * The `parse()` method expects tokenized, uppercased input and uses the
 * provided `Parser` helpers for validation and numeric parsing.
 */
class VoltageSource : public CircuitElement
{
   public:
    /**
     * @brief Construct an independent voltage source.
     *
     * @param name Unique element name (e.g., "V1").
     * @param nodeA Positive terminal node name.
     * @param nodeB Negative terminal node name.
     * @param value Voltage value in volts.
     */
    VoltageSource(const std::string &name, const std::string &nodeA,
                  const std::string &nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
        type = ElementType::V;
    }

    /**
     * @brief Stamp the voltage source into the MNA matrix and RHS.
     *
     * Voltage sources are always group-2 elements in this simulator: their
     * currents are explicit unknowns keyed by the element name in the index
     * map. The stamp inserts the KCL couplings and the element equation row
     * that enforces the voltage difference:
     *
     *   - KCL at nodeA/nodeB includes ±I_source contributions.
     *   - The element equation row is: V(nodeA) - V(nodeB) = value.
     *
     * Special-case handling is provided when either terminal is ground ("0"):
     * then only the non-ground node appears in the nodal coupling entries.
     *
     * @param mna Row-major square MNA matrix (modified in-place).
     * @param rhs RHS vector (modified in-place to include independent source).
     * @param indexMap Mapping from node/element names to matrix indices; must
     *                 include an index for `this->name` (the current variable).
     */
    virtual void stamp(std::vector<std::vector<double>> &mna,
                       std::vector<double> &rhs,
                       std::map<std::string, int> &indexMap) override;

    /**
     * @brief Parse an independent voltage source from tokenized netlist input.
     *
     * Expected tokenization (tokens are uppercased by the parser):
     *   Vname nodeA nodeB value
     *
     * The function performs:
     *  - token count validation via `parser.validateTokens()`,
     *  - node validation via `parser.validateNodes()`,
     *  - numeric value parsing via `parser.parseValue()`.
     *
     * On success returns a shared_ptr owning a new `VoltageSource` instance.
     * On failure, the helper prints diagnostics to stderr and returns
     * `nullptr`.
     *
     * @param parser Reference to the Parser instance (provides helpers).
     * @param tokens Uppercased token vector for the current line.
     * @param lineNumber Original netlist line number for diagnostic messages.
     * @return std::shared_ptr<CircuitElement> Owning pointer to the constructed
     *         element or nullptr on parse error.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser &parser, const std::vector<std::string> &tokens, int lineNumber);
};
