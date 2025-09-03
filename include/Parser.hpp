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
};
