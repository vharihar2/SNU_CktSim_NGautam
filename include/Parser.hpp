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
 * @brief Defines the Parser class for reading and processing circuit netlists.
 *
 * This file contains the definition of the Parser class, which is responsible
 * for reading a netlist file, parsing circuit elements, and storing them for
 * further analysis. It also defines the ElementCounts struct for tracking
 * element statistics.
 */

#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "CircuitElement.hpp"

// Forward Declaration
class CircuitElement;
enum class SolverDirectiveType;

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

/**
 * @struct ElementCounts
 * @brief Tracks the count of each type of circuit element encountered during
 * parsing.
 */
struct ElementCounts
{
    int resistorCount = 0; /**< Number of resistors parsed */
    int voltageSourceCount =
        0; /**< Number of independent voltage sources parsed */
    int currentSourceCount =
        0;                  /**< Number of independent current sources parsed */
    int capacitorCount = 0; /**< Number of capacitors parsed */
    int inductorCount = 0;  /**< Number of inductors parsed */
    int depVoltageSourceCount =
        0; /**< Number of dependent voltage sources parsed */
    int depCurrentSourceCount =
        0; /**< Number of dependent current sources parsed */
};

/**
 * @class Parser
 * @brief Parses a netlist file and stores the circuit elements for analysis.
 *
 * The Parser class reads a netlist file, parses each line into circuit
 * elements, and stores them in a vector. It also tracks node names and group_2
 * circuit element names for further processing and analysis.
 */
class Parser
{
   public:
    /**
     * @brief Stores the parsed circuit elements as a vector of shared pointers.
     */
    std::vector<std::shared_ptr<CircuitElement>> circuitElements;
    /**
     * @brief Stores all node names and group_2 circuit element names.
     */
    std::set<std::string> nodes_group2;

    /**
     * @brief Parses the netlist file and populates circuitElements and
     * nodes_group2.
     *
     * @param file The name of the netlist file to parse.
     * @param directive This updates the variable in the calling function to
     * reflect the type of simulation
     *
     * @return The number of errors encountered in the netlist.
     */
    int parse(const std::string& file, SolverDirectiveType& directive);

    /**
     * @brief Prints the vector containing the parsed circuit elements.
     */
    void printParser();

    /**
     * @brief Validates the number of tokens in a parsed line.
     * @param tokens The vector of tokens from the line.
     * @param lineNumber The line number in the netlist file.
     * @param expectedSize The expected number of tokens.
     * @return True if the number of tokens matches expectedSize, false
     * otherwise.
     */
    bool validateTokens(const std::vector<std::string>& tokens,
                        int expectedSize, int lineNumber);

    /**
     * @brief Parses a value string into a double.
     * @param valueStr The string representing the value.
     * @param lineNumber The line number in the netlist file.
     * @param valid Reference to a bool set to true if parsing succeeds, false
     * otherwise.
     * @return The parsed double value.
     */
    double parseValue(const std::string& valueStr, int lineNumber, bool& valid);

    /**
     * @brief Validates the node names for a circuit element.
     * @param nodeA The name of the first node.
     * @param nodeB The name of the second node.
     * @param lineNumber The line number in the netlist file.
     * @return True if the nodes are valid, false otherwise.
     */
    bool validateNodes(const std::string& nodeA, const std::string& nodeB,
                       int lineNumber);

    /**
     * @brief Prints the counts of each type of circuit element parsed.
     */
    void printElementCounts() const;

    /**
     * @brief Adds stability resistors to the circuit for numerical stability.
     */
    void addStabilityResistors();

    double tranStep = 0.0;
    double tranStop = 0.0;

   private:
    /**
     * @brief Map from element names to their corresponding CircuitElement
     * pointers.
     */
    std::map<std::string, std::shared_ptr<CircuitElement>> elementMap;
    /**
     * @brief Struct for tracking the count of each element type.
     */
    ElementCounts elementCounts;
    /**
     * @brief Value used for stability resistors.
     */
    constexpr static double STABILITY_RESISTOR_VALUE = 1e12;

    // Parsing functions for each element type
    /**
     * @brief Parses a resistor element from tokens.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     */
    void parseResistor(const std::vector<std::string>& tokens, int lineNumber);
    /**
     * @brief Parses a voltage source element from tokens.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     */
    void parseVoltageSource(const std::vector<std::string>& tokens,
                            int lineNumber);
    /**
     * @brief Parses a current source element from tokens.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     */
    void parseCurrentSource(const std::vector<std::string>& tokens,
                            int lineNumber);
    /**
     * @brief Parses a capacitor element from tokens.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     */
    void parseCapacitor(const std::vector<std::string>& tokens, int lineNumber);
    /**
     * @brief Parses an inductor element from tokens.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     */
    void parseInductor(const std::vector<std::string>& tokens, int lineNumber);
    /**
     * @brief Parses a dependent voltage source element from tokens.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     */
    void parseDependentVoltageSource(const std::vector<std::string>& tokens,
                                     int lineNumber);
    /**
     * @brief Parses a dependent current source element from tokens.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     */
    void parseDependentCurrentSource(const std::vector<std::string>& tokens,
                                     int lineNumber);

    // Additional utility functions as needed
};
