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
 * @file Resistor.cpp
 * @brief Implementation of the `Resistor` element declared in Resistor.hpp.
 *
 * This file provides the concrete parsing helper and MNA stamping routine for
 * the resistor element. Detailed API documentation lives in the header; the
 * implementation here contains concise comments describing algorithmic
 * decisions and any non-obvious behavior.
 *
 * Implementation notes:
 *  - `parse()` validates token count, node names and numeric values using the
 *    `Parser` helpers. On parse error it emits diagnostics to `stderr` and
 *    returns `nullptr`.
 *  - `stamp()` writes either a standard nodal conductance stamp (Group::G1)
 *    or the MNA coupling rows/columns for group-2 elements (Group::G2) where
 *    the element current is an explicit unknown.
 */

#include "Resistor.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "CircuitElement.hpp"
#include "Parser.hpp"

std::shared_ptr<CircuitElement> Resistor::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Validate token count: allowable forms are
    //   Rname nodeA nodeB value
    //   Rname nodeA nodeB value <optional-group>
    if (!parser.validateTokens(tokens, 4, lineNumber) &&
        !parser.validateTokens(tokens, 5, lineNumber)) {
        std::cerr << "Error: Invalid resistor definition at line " << lineNumber
                  << std::endl;
        return nullptr;
    }

    // Ensure the two terminals are distinct
    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Resistor nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Parse resistance value using the parser's strict SPICE-style parser
    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for resistor value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Construct element and set default metadata (grouping may be adjusted by
    // the parser/driver later)
    auto element =
        std::make_shared<Resistor>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::R;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

void Resistor::stamp(std::vector<std::vector<double>>& mna,
                     std::vector<double>& rhs,
                     std::map<std::string, int>& indexMap)
{
    // Group 1: standard nodal conductance stamp (no extra current unknown)
    if (group == Group::G1) {
        if (nodeA == "0") {
            // Single diagonal entry when one terminal is ground
            int vminus = indexMap[nodeB];
            mna[vminus][vminus] += 1.0 / value;
        } else if (nodeB == "0") {
            int vplus = indexMap[nodeA];
            mna[vplus][vplus] += 1.0 / value;
        } else {
            // Full 2x2 conductance contribution
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            double conductance = 1.0 / value;
            mna[vplus][vplus] += conductance;
            mna[vplus][vminus] -= conductance;
            mna[vminus][vplus] -= conductance;
            mna[vminus][vminus] += conductance;
        }
    }
    // Group 2: element current is an explicit unknown => MNA coupling rows/cols
    else {
        // Retrieve element current index (must be present in indexMap)
        int i = indexMap[name];

        if (nodeA == "0") {
            int vminus = indexMap[nodeB];
            // KCL at nodeB: -I_element
            mna[vminus][i] -= 1.0;
            // Equation for element I-V relation: -V(nodeB) - R*I = 0  (signs
            // per conv)
            mna[i][vminus] -= 1.0;
            mna[i][i] -= value;
        } else if (nodeB == "0") {
            int vplus = indexMap[nodeA];
            mna[vplus][i] += 1.0;
            mna[i][vplus] += 1.0;
            mna[i][i] -= value;
        } else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            // KCL couplings
            mna[vplus][i] += 1.0;
            mna[vminus][i] -= 1.0;
            // Element equation coupling
            mna[i][vplus] += 1.0;
            mna[i][vminus] -= 1.0;
            // Element's own I-V relation (Ohm's law): -R on diagonal
            mna[i][i] -= value;
        }
    }
}
