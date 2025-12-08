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
 * @file CurrentSource.cpp
 * @brief Implementation of the `CurrentSource` class declared in
 * CurrentSource.hpp.
 *
 * Provides the parsing factory and DC stamping behavior for an independent
 * current source element. For DC Modified Nodal Analysis (MNA) an ideal
 * current source contributes only to the RHS current injection vector.
 */

#include "CurrentSource.hpp"

#include <memory>
#include <string>
#include <vector>

#include "Parser.hpp"

std::shared_ptr<CircuitElement> CurrentSource::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Accept either 4 tokens (standard) or 5 tokens (optional group)
    if (!parser.validateTokens(tokens, 4, lineNumber) &&
        !parser.validateTokens(tokens, 5, lineNumber)) {
        std::cerr << "Error: Invalid current source definition at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Current source nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for current source value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element =
        std::make_shared<CurrentSource>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::I;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

void CurrentSource::stamp(std::vector<std::vector<double>>& mna,
                          std::vector<double>& rhs,
                          std::map<std::string, int>& indexMap)
{
    // DC stamping: ideal current source injects current into RHS only.
    // Mark unused parameter to avoid warnings.
    (void)mna;

    if (nodeA != "0") {
        int vplus = indexMap[nodeA];
        rhs[vplus] -= value;
    }
    if (nodeB != "0") {
        int vminus = indexMap[nodeB];
        rhs[vminus] += value;
    }
}
