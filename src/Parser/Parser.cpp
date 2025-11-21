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
 * @file Parser.cpp
 *
 * @brief Contains the implementation of the Parser class
 */

#include "Parser.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <ostream>
#include <sstream>

#include "Capacitor.hpp"
#include "CircuitElement.hpp"
#include "CurrentSource.hpp"
#include "DependentCurrentSource.hpp"
#include "DependentVoltageSource.hpp"
#include "Inductor.hpp"
#include "Resistor.hpp"
#include "Solver.hpp"
#include "VoltageSource.hpp"

int Parser::parse(const std::string& fileName, SolverDirectiveType& directive)
{
    std::ifstream fileStream(fileName);
    if (!fileStream) {
        std::cerr << "Error: Netlist not available in the project directory"
                  << std::endl;
        return 1;
    }

    std::string line;
    int lineNumber = 1;
    int errorCount = 0;

    // Clear previous data
    circuitElements.clear();
    nodes_group2.clear();
    elementMap.clear();

    while (std::getline(fileStream, line)) {
        lineNumber++;

        // Convert line to uppercase for uniformity
        std::transform(line.begin(), line.end(), line.begin(), ::toupper);

        // Tokenize line
        std::stringstream ss(line);
        std::vector<std::string> tokens;
        std::string buf;
        while (ss >> buf) tokens.push_back(buf);

        // Skip empty lines and comments
        if (tokens.empty() || tokens[0].find("%") == 0) continue;

        std::shared_ptr<CircuitElement> element = nullptr;
        // Dispatch to element-specific parse functions
        if (tokens[0].find("R") == 0) {
            ++elementCounts.resistorCount;
            element = Resistor::parse(*this, tokens, lineNumber);
            if (element && element->getGroup() == Group::G2) {
                nodes_group2.insert(element->getName());
            }
        } else if (tokens[0].find("V") == 0 && tokens[0].find("VC") != 0) {
            ++elementCounts.voltageSourceCount;
            element = VoltageSource::parse(*this, tokens, lineNumber);
            if (element) {
                nodes_group2.insert(element->getName());
            }
        } else if (tokens[0].find("I") == 0 && tokens[0].find("IC") != 0) {
            ++elementCounts.currentSourceCount;
            element = CurrentSource::parse(*this, tokens, lineNumber);
        } else if (tokens[0].find("C") == 0) {
            ++elementCounts.capacitorCount;
            element = Capacitor::parse(*this, tokens, lineNumber);
        } else if (tokens[0].find("L") == 0) {
            ++elementCounts.inductorCount;
            element = Inductor::parse(*this, tokens, lineNumber);
            if (element) {
                nodes_group2.insert(element->getName());
            }
        } else if (tokens[0].find("VC") == 0) {
            ++elementCounts.depVoltageSourceCount;
            element = DependentVoltageSource::parse(*this, tokens, lineNumber);
            if (element) {
                nodes_group2.insert(element->getName());
            }
        } else if (tokens[0].find("IC") == 0) {
            ++elementCounts.depCurrentSourceCount;
            element = DependentCurrentSource::parse(*this, tokens, lineNumber);

        } else if (tokens[0].find(".OP") == 0) {
            // Operational point directive
            if (directive != SolverDirectiveType::NONE) {
                std::cerr << "Warning: Multiple directives found. Using the "
                             "first one."
                          << std::endl;
                ++errorCount;
            }
            directive = SolverDirectiveType::OPERATING_POINT;

        } else if (tokens[0].find(".TRAN") == 0) {
            // Transient directive: .TRAN <Tstep> <Tstop>
            if (directive != SolverDirectiveType::NONE) {
                std::cerr << "Warning: Multiple directives found. Using the "
                             "first one."
                          << std::endl;
                ++errorCount;
            }
            // Expect exactly 3 tokens: .TRAN Tstep Tstop
            if (tokens.size() != 3) {
                std::cerr << "Line " << lineNumber
                          << ": .TRAN expects 2 arguments: <Tstep> <Tstop>"
                          << std::endl;
                ++errorCount;
            } else {
                bool validStep = false;
                bool validStop = false;
                double step = parseValue(tokens[1], lineNumber, validStep);
                double stop = parseValue(tokens[2], lineNumber, validStop);
                if (!validStep || !validStop) {
                    std::cerr << "Line " << lineNumber
                              << ": Invalid numeric argument in .TRAN directive"
                              << std::endl;
                    ++errorCount;
                } else if (step <= 0.0 || stop <= 0.0) {
                    std::cerr << "Line " << lineNumber
                              << ": .TRAN arguments must be positive numbers"
                              << std::endl;
                    ++errorCount;
                } else if (step > stop) {
                    std::cerr << "Line " << lineNumber
                              << ": .TRAN step must be <= stop time"
                              << std::endl;
                    ++errorCount;
                } else {
                    // Store parsed values in the parser instance
                    this->tranStep = step;
                    this->tranStop = stop;
                    directive = SolverDirectiveType::TRANSIENT;
                }
            }
        } else {
            std::cerr << "Error: Unknown element at line " << lineNumber << ": "
                      << line << std::endl;
            ++errorCount;
        }

        if (element) {
            // if (element->getGroup() == Group::G2) {
            //     nodes_group2.insert(element->getName());
            // }
            nodes_group2.insert(element->getNodeA());
            nodes_group2.insert(element->getNodeB());

            circuitElements.push_back(element);
            elementMap[element->getName()] = element;
        }
    }

    // Resolve controlling_element pointers for dependent sources
    // for (auto& elem : circuitElements) {
    //     if (elem->controlling_variable != ControlVariable::none &&
    //         elem->controlling_element) {
    //         auto it = elementMap.find(elem->controlling_element->name);
    //         if (it != elementMap.end()) {
    //             elem->controlling_element = it->second;
    //             // If controlling variable is current, referenced element
    //             must
    //             // be group 2
    //             if (elem->controlling_variable == ControlVariable::i &&
    //                 elem->controlling_element->group != Group::G2) {
    //                 std::cerr << "Warning: Referenced element "
    //                           << elem->controlling_element->name
    //                           << " must be in group 2 as its current
    //                           variable
    //                           "
    //                              "is required by "
    //                           << elem->name << std::endl;
    //                 elem->controlling_element->group = Group::G2;
    //                 nodes_group2.insert(elem->controlling_element->name);
    //             }
    //         } else {
    //             std::cerr << "Error: Referenced element "
    //                       << elem->controlling_element->name
    //                       << " (referenced by " << elem->name
    //                       << ") is not present in the netlist" <<
    //                       std::endl;
    //             errorCount++;
    //         }
    //     }
    // }

    errorCount += CircuitElement::resolveDependencies(circuitElements,
                                                      elementMap, nodes_group2);
    // Check for ground node
    if (nodes_group2.find("0") == nodes_group2.end()) {
        std::cerr << "Error: Circuit must contain ground (0)" << std::endl;
        errorCount++;
    }

    // Print summary (optional)
    std::cout << "\nTotal elements in the circuit: " << circuitElements.size()
              << std::endl;

    printElementCounts();
    addStabilityResistors();
    return errorCount;
}

void Parser::printParser()
{
    // for (std::shared_ptr<CircuitElement> circuitElement :
    // circuitElements)
    //     if (circuitElement->type == ElementType::V ||
    //         circuitElement->type == ElementType::I ||
    //         circuitElement->type == ElementType::R)
    //         cout << circuitElement->name + " " + circuitElement->nodeA +
    //         " "
    //         +
    //                     circuitElement->nodeB + " "
    //              << circuitElement->value << " " << circuitElement->group
    //              << endl;
    //     else
    //         cout << circuitElement->name + " " + circuitElement->nodeA +
    //         " "
    //         +
    //                     circuitElement->nodeB + " "
    //              << circuitElement->value << " " << circuitElement->group
    //              << " "
    //              << circuitElement->controlling_variable << " "
    //              << circuitElement->controlling_element->name + " " +
    //                     circuitElement->controlling_element->nodeA + " "
    //                     + circuitElement->controlling_element->nodeB
    //              << endl;
}
bool Parser::validateTokens(const std::vector<std::string>& tokens,
                            int expectedSize, int lineNumber)
{
    if (tokens.size() != expectedSize) {
        std::cerr << "Line " << lineNumber << ": Expected " << expectedSize
                  << " tokens, got " << tokens.size() << std::endl;
        return false;
    }
    return true;
}

bool Parser::validateNodes(const std::string& nodeA, const std::string& nodeB,
                           int lineNumber)
{
    if (nodeA == nodeB) {
        std::cerr << "Line " << lineNumber
                  << ": NodeA and NodeB cannot be the same (" << nodeA << ")"
                  << std::endl;
        return false;
    }
    return true;
}

double Parser::parseValue(const std::string& valueStr, int lineNumber,
                          bool& valid)
{
    try {
        double value = std::stod(valueStr);
        valid = true;
        return value;
    } catch (const std::exception&) {
        std::cerr << "Line " << lineNumber << ": Invalid value '" << valueStr
                  << "'" << std::endl;
        valid = false;
        return 0.0;
    }
}

// void Parser::parseResistor(const std::vector<std::string>& tokens,
//                            int lineNumber)
// {
//     // Validate token count
//     if (!validateTokens(tokens, 4, lineNumber) &&
//         !validateTokens(tokens, 5, lineNumber)) {
//         std::cerr << "Error: Invalid resistor definition at line " <<
//         lineNumber
//                   << std::endl;
//         return;
//     }
//
//     // Validate nodes
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Resistor nodes cannot be the same at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     // Parse value
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for resistor value at line
//         "
//                   << lineNumber << std::endl;
//         value = 1;  // Default to 1 ohm if invalid
//     }
//
//     // Create resistor element
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::R;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     // Update nodes_group2 if group is G2
//     if (temp->group == Group::G2) {
//         nodes_group2.insert(temp->name);
//     }
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     // Add to circuitElements and elementMap
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
//
// void Parser::parseVoltageSource(const std::vector<std::string>& tokens,
//                                 int lineNumber)
// {
//     // Validate token count
//     if (!validateTokens(tokens, 4, lineNumber)) {
//         std::cerr << "Error: Invalid voltage source definition at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     // Validate nodes
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Voltage source nodes cannot be the same at
//         line
//         "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     // Parse value
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for voltage source value at
//         line "
//                   << lineNumber << std::endl;
//         value = 1;  // Default to 1V if invalid
//     }
//
//     // Create voltage source element
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::V;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     // Update nodes_group2
//     nodes_group2.insert(temp->name);
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     // Add to circuitElements and elementMap
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
//
// void Parser::parseCurrentSource(const std::vector<std::string>& tokens,
//                                 int lineNumber)
// {
//     // Valid token counts: 4 or 5 (for optional group)
//     if (!validateTokens(tokens, 4, lineNumber) &&
//         !validateTokens(tokens, 5, lineNumber)) {
//         std::cerr << "Error: Invalid current source definition at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Current source nodes cannot be the same at
//         line
//         "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for current source value at
//         line "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::I;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
// void Parser::parseCapacitor(const std::vector<std::string>& tokens,
//                             int lineNumber)
// {
//     // Valid token counts: 4 or 5 (for optional group)
//     if (!validateTokens(tokens, 4, lineNumber) &&
//         !validateTokens(tokens, 5, lineNumber)) {
//         std::cerr << "Error: Invalid capacitor definition at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Capacitor nodes cannot be the same at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for capacitor value at line
//         "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::C;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
// void Parser::parseInductor(const std::vector<std::string>& tokens,
//                            int lineNumber)
// {
//     // Inductors are always group 2
//     if (!validateTokens(tokens, 4, lineNumber)) {
//         std::cerr << "Error: Invalid inductor definition at line " <<
//         lineNumber
//                   << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Inductor nodes cannot be the same at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for inductor value at line
//         "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::L;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     nodes_group2.insert(temp->name);
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
//
// void Parser::parseDependentVoltageSource(const std::vector<std::string>&
// tokens,
//                                          int lineNumber)
// {
//     // Must have at least 6 tokens: VC name nodeA nodeB value
//     // controlling_variable controlling_element
//     if (!validateTokens(tokens, 6, lineNumber)) {
//         std::cerr
//             << "Error: Invalid dependent voltage source definition at
//             line "
//             << lineNumber << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Dependent voltage source nodes cannot be the
//         same "
//                      "at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for dependent voltage
//         source "
//                      "value at line "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::Vc;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//
//     // Controlling variable: must be "V" or "I"
//     if (tokens[4] == "V") {
//         temp->controlling_variable = ControlVariable::v;
//     } else if (tokens[4] == "I") {
//         temp->controlling_variable = ControlVariable::i;
//     } else {
//         std::cerr << "Error: Illegal controlling variable argument at
//         line "
//                   << lineNumber << std::endl;
//         temp->controlling_variable = ControlVariable::none;
//     }
//
//     // Cascading check: controlling element cannot be VC or IC
//     if (tokens[5].find("IC") == 0 || tokens[5].find("VC") == 0) {
//         std::cerr << "Error: Controlled source " << tokens[0]
//                   << " cannot be cascaded at line " << lineNumber <<
//                   std::endl;
//         temp->controlling_variable = ControlVariable::none;
//         temp->controlling_element = nullptr;
//     } else {
//         temp->controlling_element = std::make_shared<CircuitElement>();
//         temp->controlling_element->name = tokens[5];
//     }
//
//     temp->processed = false;
//
//     nodes_group2.insert(temp->name);
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
// }
//
// void Parser::parseDependentCurrentSource(const std::vector<std::string>&
// tokens,
//                                          int lineNumber)
// {
//     // Must have at least 6 tokens: IC name nodeA nodeB value
//     // controlling_variable controlling_element
//     if (!validateTokens(tokens, 6, lineNumber)) {
//         std::cerr
//             << "Error: Invalid dependent current source definition at
//             line "
//             << lineNumber << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Dependent current source nodes cannot be the
//         same "
//                      "at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for dependent current
//         source "
//                      "value at line "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::Ic;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//
//     // Controlling variable: must be "V" or "I"
//     if (tokens[4] == "V") {
//         temp->controlling_variable = ControlVariable::v;
//     } else if (tokens[4] == "I") {
//         temp->controlling_variable = ControlVariable::i;
//     } else {
//         std::cerr << "Error: Illegal controlling variable argument at
//         line "
//                   << lineNumber << std::endl;
//         temp->controlling_variable = ControlVariable::none;
//     }
//
//     // Cascading check: controlling element cannot be VC or IC
//     if (tokens[5].find("IC") == 0 || tokens[5].find("VC") == 0) {
//         std::cerr << "Error: Controlled source " << tokens[0]
//                   << " cannot be cascaded at line " << lineNumber <<
//                   std::endl;
//         temp->controlling_variable = ControlVariable::none;
//         temp->controlling_element = nullptr;
//     } else {
//         temp->controlling_element = std::make_shared<CircuitElement>();
//         temp->controlling_element->name = tokens[5];
//     }
//
//     temp->processed = false;
//
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
// }

void Parser::printElementCounts() const
{
    std::cout << "Total Resistors: " << elementCounts.resistorCount
              << std::endl;
    std::cout << "Total Voltage Sources: " << elementCounts.voltageSourceCount
              << std::endl;
    std::cout << "Total Current Sources: " << elementCounts.currentSourceCount
              << std::endl;
    std::cout << "Total Capacitors: " << elementCounts.capacitorCount
              << std::endl;
    std::cout << "Total Inductors: " << elementCounts.inductorCount
              << std::endl;
    std::cout << "Total Dependent Voltage Sources: "
              << elementCounts.depVoltageSourceCount << std::endl;
    std::cout << "Total Dependent Current Sources: "
              << elementCounts.depCurrentSourceCount << std::endl;
}

void Parser::addStabilityResistors()
{
    int stabilityResistorIndex = 1;
    for (const auto& node : nodes_group2) {
        if (node == "0") continue;  // Skip ground
        std::string resistorName = "RSTAB_" + node;
        auto temp = std::make_shared<Resistor>(resistorName, node, "0",
                                               STABILITY_RESISTOR_VALUE);
        circuitElements.push_back(temp);
        elementMap[temp->getName()] = temp;
        // Optionally: nodes_group2.insert(node); // Already present
        stabilityResistorIndex++;
    }
}
