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
 * @file Parser.hpp
 *
 * @brief Contains the definition of the Parser class
 */

#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "CircuitElement.hpp"

/**
 * @class Parser
 *
 * @brief Parses the netlist file and stores the circuit elements
 *
 * The Parser class reads the netlist file and stores the circuit elements in a
 * vector. It also stores the node names and group_2 circuit element names in a
 * set for further processing.
 *
 * */

struct ElementCounts
{
    int resistorCount = 0;
    int voltageSourceCount = 0;
    int currentSourceCount = 0;
    int capacitorCount = 0;
    int inductorCount = 0;
    int depVoltageSourceCount = 0;
    int depCurrentSourceCount = 0;
};

class Parser
{
   public:
    std::vector<std::shared_ptr<CircuitElement>>
        circuitElements; /**< Stores the circuit elements in form of a vector */
    std::set<std::string> nodes_group2; /**< Stores all node names and group_2
                                         circuit element names*/

    /**
     * @brief		Parses the file (netlist) into a vector
     *
     * @param	file The name of the file
     *
     * @return		number of errors in the netlist
     */
    int parse(const std::string& file);

    /**
     * @brief		Prints the vectors which contains the circuit elements
     *
     */
    void printParser();

   private:
    // element map
    std::map<std::string, std::shared_ptr<CircuitElement>> elementMap;
    ElementCounts elementCounts;

    // One function per element type
    void parseResistor(const std::vector<std::string>& tokens, int lineNumber);
    void parseVoltageSource(const std::vector<std::string>& tokens,
                            int lineNumber);
    void parseCurrentSource(const std::vector<std::string>& tokens,
                            int lineNumber);
    void parseCapacitor(const std::vector<std::string>& tokens, int lineNumber);
    void parseInductor(const std::vector<std::string>& tokens, int lineNumber);
    void parseDependentVoltageSource(const std::vector<std::string>& tokens,
                                     int lineNumber);
    void parseDependentCurrentSource(const std::vector<std::string>& tokens,
                                     int lineNumber);

    // utility functions
    bool validateTokens(const std::vector<std::string>& tokens, int lineNumber,
                        int expectedSize);
    double parseValue(const std::string& valueStr, int lineNumber, bool& valid);
    bool validateNodes(const std::string& nodeA, const std::string& nodeB,
                       int lineNumber);
    void printElementCounts() const;
};
