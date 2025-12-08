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
 * @file VoltageSource.cpp
 * @brief Implementation of the `VoltageSource` element.
 *
 * This file provides the concrete parsing helper (`VoltageSource::parse`)
 * used by the netlist `Parser` and the MNA stamping routine
 * (`VoltageSource::stamp`) which inserts the independent source's contributions
 * into the MNA matrix and RHS vector.
 *
 * Implementation notes (concise):
 *  - Voltage sources are treated as Group-2 elements in this simulator: their
 *    branch current is an explicit unknown keyed by the element name in the
 *    index map.
 *  - The stamp inserts KCL couplings between node voltages and the element
 *    current unknown and writes the source voltage into the element equation's
 *    RHS entry. Special-cases are handled when either terminal is ground ("0").
 *
 * Full API documentation for construction, parsing semantics and stamping
 * behavior is provided in the header `VoltageSource.hpp`.
 */

#include "VoltageSource.hpp"

#include <memory>

#include "Parser.hpp"

/**
 * @brief Parse a voltage source definition from tokenized netlist input.
 *
 * The expected form is:
 *   Vname nodeA nodeB value
 *
 * This helper validates token count and node names and uses the parser's
 * strict SPICE-like numeric parser to interpret the source value. On failure
 * it prints diagnostics to stderr and returns nullptr.
 */
std::shared_ptr<CircuitElement> VoltageSource::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Validate token count (exactly 4 tokens expected for a basic DC source)
    if (!parser.validateTokens(tokens, 4, lineNumber)) {
        std::cerr << "Error: Invalid voltage source definition at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Validate node names (terminals must be distinct)
    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Voltage source nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Parse numeric value with suffix handling (e.g., 5V, 1K etc.)
    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for voltage source value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Construct element and set standard metadata.
    // Group is G2 because an independent voltage source introduces a current
    // unknown in the MNA formulation.
    std::shared_ptr<VoltageSource> element =
        std::make_shared<VoltageSource>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::V;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

/**
 * @brief Stamp the independent voltage source into the MNA matrix and RHS.
 *
 * MNA stamping pattern (element current unknown = I_e):
 *
 *   KCL rows:
 *     - At nodeA: +I_e (if nodeA != 0)
 *     - At nodeB: -I_e (if nodeB != 0)
 *
 *   Element equation row (for I_e):
 *     V(nodeA) - V(nodeB) = value  -> coefficients +1 and -1 on node voltages,
 *     RHS receives the source value.
 *
 * The implementation handles the cases where one terminal is ground ('0')
 * to avoid writing nodal coupling entries for ground.
 */
void VoltageSource::stamp(std::vector<std::vector<double>>& mna,
                          std::vector<double>& rhs,
                          std::map<std::string, int>& indexMap)
{
    // Independent Voltage Source (always Group 2)

    // Case: nodeA is ground -> only nodeB has a nodal equation coupling
    if (nodeA.compare("0") == 0) {
        int vminus = indexMap[nodeB];
        int i = indexMap[name];  // element current unknown index
        // KCL at nodeB: -I_e
        mna[vminus][i] += -1.0;
        // Element equation: -V(nodeB) = -value  (rearranged sign convention)
        mna[i][vminus] += -1.0;
        // RHS for element equation gets +value
        rhs[i] += value;
    }
    // Case: nodeB is ground -> only nodeA has a nodal equation coupling
    else if (nodeB.compare("0") == 0) {
        int vplus = indexMap[nodeA];
        int i = indexMap[name];
        // KCL at nodeA: +I_e
        mna[vplus][i] += 1.0;
        // Element equation: +V(nodeA) = value
        mna[i][vplus] += 1.0;
        rhs[i] += value;
    }
    // General case: both terminals are non-ground
    else {
        int vplus = indexMap[nodeA];
        int vminus = indexMap[nodeB];
        int i = indexMap[name];
        // KCL: nodeA receives +I_e, nodeB receives -I_e
        mna[vplus][i] += 1.0;
        mna[vminus][i] += -1.0;
        // Element equation: V(nodeA) - V(nodeB) = value
        mna[i][vplus] += 1.0;
        mna[i][vminus] += -1.0;
        rhs[i] += value;
    }
}
