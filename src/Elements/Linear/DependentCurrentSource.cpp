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
 * @file DependentCurrentSource.cpp
 * @brief Implementation of DependentCurrentSource parsing and MNA stamping.
 *
 * Provides the parse factory and concise, focused stamping logic for
 * dependent current sources (VCCS / CCCS). Comments are short and describe
 * the implementation, not developer notes.
 */

#include "DependentCurrentSource.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Parser.hpp"

std::shared_ptr<CircuitElement> DependentCurrentSource::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Expect tokens: name nodeA nodeB value <V|I> controllingElement
    if (!parser.validateTokens(tokens, 6, lineNumber)) {
        std::cerr
            << "Error: Invalid dependent current source definition at line "
            << lineNumber << std::endl;
        return nullptr;
    }

    // Nodes must differ
    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Dependent current source nodes cannot be the same "
                     "at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Parse coefficient value
    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for dependent current source "
                     "value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element = std::make_shared<DependentCurrentSource>(
        tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::Ic;
    element->group = Group::G2;

    // Controlling variable: 'V' or 'I'
    if (tokens[4] == "V") {
        element->controlling_variable = ControlVariable::v;
    } else if (tokens[4] == "I") {
        element->controlling_variable = ControlVariable::i;
    } else {
        std::cerr << "Error: Illegal controlling variable argument at line "
                  << lineNumber << std::endl;
        element->controlling_variable = ControlVariable::none;
    }

    // Prevent cascading controlled sources (names starting with IC/VC)
    if (tokens[5].find("IC") == 0 || tokens[5].find("VC") == 0) {
        std::cerr << "Error: Controlled source " << tokens[0]
                  << " cannot be cascaded at line " << lineNumber << std::endl;
        element->controlling_variable = ControlVariable::none;
        element->controlling_element = nullptr;
    } else {
        element->controlling_element = nullptr;
    }

    return element;
}

void DependentCurrentSource::stamp(std::vector<std::vector<double>>& mna,
                                   std::vector<double>& /*rhs*/,
                                   std::map<std::string, int>& indexMap)
{
    // Dependent current sources alter the MNA matrix entries according to the
    // control type. RHS is unused for this formulation.
    // Marked unused in signature above.

    // Current-controlled current source (CCCS)
    if (controlling_variable == ControlVariable::i) {
        if (nodeA == "0") {
            int vminus = indexMap[nodeB];
            int i = indexMap[controlling_element->getName()];
            mna[vminus][i] += -value;
        } else if (nodeB == "0") {
            int vplus = indexMap[nodeA];
            int i = indexMap[controlling_element->getName()];
            mna[vplus][i] += value;
        } else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            int i = indexMap[controlling_element->getName()];
            mna[vplus][i] += value;
            mna[vminus][i] += -value;
        }
    }
    // Voltage-controlled current source (VCCS)
    else {
        if (nodeA == "0") {
            int vminus = indexMap[nodeB];
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vminus][vxminus] += value;
            } else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                mna[vminus][vxplus] += -value;
            } else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vminus][vxplus] += -value;
                mna[vminus][vxminus] += value;
            }
        } else if (nodeB == "0") {
            int vplus = indexMap[nodeA];
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][vxminus] += -value;
            } else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                mna[vplus][vxplus] += value;
            } else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][vxplus] += value;
                mna[vplus][vxminus] += -value;
            }
        } else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][vxminus] += -value;
                mna[vminus][vxminus] += value;
            } else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                mna[vplus][vxplus] += value;
                mna[vminus][vxplus] += -value;
            } else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][vxplus] += value;
                mna[vplus][vxminus] += -value;
                mna[vminus][vxplus] += -value;
                mna[vminus][vxminus] += value;
            }
        }
    }
}
