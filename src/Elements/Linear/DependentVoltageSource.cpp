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
 * @file DependentVoltageSource.cpp
 * @brief Implementation of the DependentVoltageSource class declared in the
 * header.
 *
 * This file provides the parsing factory and MNA stamping logic for controlled
 * voltage sources (VCVS and CCVS). Comments are concise and focus on the
 * implementation details required for maintainers.
 */

#include "DependentVoltageSource.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Parser.hpp"

std::shared_ptr<CircuitElement> DependentVoltageSource::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Expect at least 6 tokens:
    // VCname nodeA nodeB value <V|I> controllingElement
    if (!parser.validateTokens(tokens, 6, lineNumber)) {
        std::cerr
            << "Error: Invalid dependent voltage source definition at line "
            << lineNumber << std::endl;
        return nullptr;
    }

    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Dependent voltage source nodes cannot be the same "
                     "at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for dependent voltage source "
                     "value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element = std::make_shared<DependentVoltageSource>(
        tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::Vc;
    element->group = Group::G2;

    // Parse controlling variable ("V" or "I")
    if (tokens[4] == "V") {
        element->controlling_variable = ControlVariable::v;
    } else if (tokens[4] == "I") {
        element->controlling_variable = ControlVariable::i;
    } else {
        std::cerr << "Error: Illegal controlling variable argument at line "
                  << lineNumber << std::endl;
        element->controlling_variable = ControlVariable::none;
    }

    // Prevent cascading controlled sources: controlling element name starting
    // with "IC" or "VC" is rejected here; pointer is resolved later.
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

void DependentVoltageSource::stamp(std::vector<std::vector<double>>& mna,
                                   std::vector<double>& /*rhs*/,
                                   std::map<std::string, int>& indexMap)
{
    // Controlled voltage sources are Group G2 and introduce an additional
    // unknown (the branch current). The stamping below follows the MNA
    // convention used across the project.

    // Current-Controlled Voltage Source (CCVS)
    if (controlling_variable == ControlVariable::i) {
        // nodeA is ground
        if (nodeA == "0") {
            int vminus = indexMap[nodeB];
            int is = indexMap[name];
            int ix = indexMap[controlling_element->getName()];
            mna[vminus][is] += -1.0;
            mna[is][vminus] += -1.0;
            mna[is][ix] += -value;
        }
        // nodeB is ground
        else if (nodeB == "0") {
            int vplus = indexMap[nodeA];
            int is = indexMap[name];
            int ix = indexMap[controlling_element->getName()];
            mna[vplus][is] += 1.0;
            mna[is][vplus] += 1.0;
            mna[is][ix] += -value;
        }
        // neither node is ground
        else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            int is = indexMap[name];
            int ix = indexMap[controlling_element->getName()];
            mna[vplus][is] += 1.0;
            mna[vminus][is] += -1.0;
            mna[is][vplus] += 1.0;
            mna[is][vminus] += -1.0;
            mna[is][ix] += -value;
        }
    }
    // Voltage-Controlled Voltage Source (VCVS)
    else {
        // nodeA is ground
        if (nodeA == "0") {
            int vminus = indexMap[nodeB];
            int i = indexMap[name];
            // controlling element source node is ground
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vminus][i] += -1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxminus] += value;
            }
            // controlling element target node is ground
            else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                mna[vminus][i] += -1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -value;
            }
            // controlling element neither node is ground
            else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vminus][i] += -1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -value;
                mna[i][vxminus] += value;
            }
        }
        // nodeB is ground
        else if (nodeB == "0") {
            int vplus = indexMap[nodeA];
            int i = indexMap[name];
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][i] += 1.0;
                mna[i][vplus] += 1.0;
                mna[i][vxminus] += value;
            } else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                mna[vplus][i] += 1.0;
                mna[i][vplus] += 1.0;
                mna[i][vxplus] += -value;
            } else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][i] += 1.0;
                mna[i][vplus] += 1.0;
                mna[i][vxplus] += -value;
                mna[i][vxminus] += value;
            }
        }
        // neither node is ground
        else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            int i = indexMap[name];
            // controlling element source node is ground
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][i] += 1.0;
                mna[vminus][i] += -1.0;
                mna[i][vplus] += 1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxminus] += value;
            }
            // controlling element target node is ground
            else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                mna[vplus][i] += 1.0;
                mna[vminus][i] += -1.0;
                mna[i][vplus] += 1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -value;
            }
            // controlling element neither node is ground
            else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][i] += 1.0;
                mna[vminus][i] += -1.0;
                mna[i][vplus] += 1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -value;
                mna[i][vxminus] += value;
            }
        }
    }
}
