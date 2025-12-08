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
 * @brief Parser for SPICE-like netlists.
 *
 * This header defines the `Parser` class which is responsible for reading a
 * textual netlist, tokenizing and expanding subcircuit instances, validating
 * and parsing element lines, and producing a collection of element objects
 * ready for circuit assembly and analysis.
 *
 * Design notes:
 *  - Tokenization preserves parenthesized groups (e.g., behavioral expressions
 *    or function arguments) and uppercases the input for consistent keyword
 *    handling.
 *  - Subcircuits declared with `.SUBCKT` ... `.ENDS` are collected and later
 *    expanded for each instance (X... syntax). Instance expansion performs
 *    port substitution and name-mangling to avoid collisions.
 *  - Numeric literals are parsed in a strict SPICE-like manner: optional
 *    suffix multipliers (T, G, MEG, K, M, U, N, P, F) are supported and the
 *    mantissa must be a well-formed floating-point literal with no trailing
 *    garbage.
 *
 * The Parser produces:
 *  - `circuitElements`: a vector of shared pointers to parsed `CircuitElement`
 *    objects (concrete element classes like `Resistor`, `Capacitor`, etc.).
 *  - `nodes_group2`: a set of node and group-2 element names used by the
 *    solver for indexing/grouping.
 *
 * Example usage:
 * @code
 * Parser p;
 * SolverDirectiveType directive = SolverDirectiveType::NONE;
 * int errors = p.parse(\"netlist.cir\", directive);
 * if (errors == 0) {
 *   // proceed with circuit analysis using p.circuitElements
 * }
 * @endcode
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
 * @struct ElementCounts
 * @brief Simple counters for diagnostics and summary reporting.
 *
 * Parser increments the appropriate counter while parsing elements; these
 * counters are printed by `printElementCounts()` to give a quick summary of
 * the parsed netlist composition.
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
 * @brief Reads and parses a SPICE-like netlist into element objects.
 *
 * The Parser supports:
 *  - Top-level directives such as `.OP` and `.TRAN`.
 *  - Subcircuit definitions (`.SUBCKT` / `.ENDS`) and instance expansion
 *    (lines starting with `X...`).
 *  - Recognition and parsing of linear and nonlinear element syntaxes,
 *    including `POLY` polynomial coefficient lists for nonlinear capacitors
 *    and inductors.
 *
 * The Parser performs error reporting to `std::cerr` and returns a non-zero
 * error count from `parse()` when problems are encountered. It also ensures
 * that the circuit contains a ground node ('0') before returning success.
 */
class Parser
{
   public:
    /**
     * @brief Parsed circuit element list.
     *
     * After `parse()` completes, this vector contains shared pointers to
     * concrete element instances (e.g., `Resistor`, `VoltageSource`,
     * `NonlinearCapacitor`, ...). The simulator will read this vector to
     * build the system matrices.
     */
    std::vector<std::shared_ptr<CircuitElement>> circuitElements;

    /**
     * @brief Set of node names and group-2 element names.
     *
     * The solver relies on this set to determine which nodes/elements belong
     * to group 2 (current-carrying elements that require additional variables
     * in the nodal analysis matrix).
     */
    std::set<std::string> nodes_group2;

    /**
     * @brief Parses a netlist file and populates internal data structures.
     *
     * This is the main entrypoint for the Parser. It reads the file, tokenizes
     * and uppercases lines, gathers subcircuit definitions, expands instances,
     * and creates element objects. Numeric parsing and token validation errors
     * are reported to stderr; the function returns the total number of
     * reported errors.
     *
     * @param file Path to the netlist file to parse.
     * @param directive Reference updated to reflect an encountered solver
     *                  directive (e.g., `.OP` or `.TRAN`). If no directive
     *                  is present, the value remains
     * `SolverDirectiveType::NONE`.
     * @return Number of errors encountered while parsing. Zero indicates a
     *         clean parse.
     */
    int parse(const std::string& file, SolverDirectiveType& directive);

    /**
     * @brief Print a human-readable listing of parsed elements.
     *
     * The current implementation prints nothing by default; this helper exists
     * for debugging and may be extended to produce a concise element listing.
     */
    void printParser();

    /**
     * @brief Validate the number of tokens in a line.
     *
     * Basic helper used by legacy/per-element parsing helpers to assert that
     * the token vector contains the expected number of fields.
     *
     * @param tokens Tokenized line.
     * @param expectedSize Expected token count.
     * @param lineNumber Associated line number (for error messages).
     * @return True if token count matches `expectedSize`, false otherwise.
     */
    bool validateTokens(const std::vector<std::string>& tokens,
                        int expectedSize, int lineNumber);

    /**
     * @brief Parse a numeric value string into a double.
     *
     * Accepts optional suffix multipliers (T, G, MEG, K, M, U, N, P, F). The
     * mantissa must be a well-formed numeric literal (std::stod must consume
     * the entire mantissa). The parser is strict and will set `valid` to false
     * for malformed inputs.
     *
     * @param valueStr Uppercased value token (e.g., \"10K\", \"3.3U\").
     * @param lineNumber Line number in the netlist (used for diagnostics).
     * @param valid Output parameter set to true when parsing succeeds.
     * @return Parsed numeric value (undefined if `valid` is false).
     */
    double parseValue(const std::string& valueStr, int lineNumber, bool& valid);

    /**
     * @brief Validate node names for a two-terminal element.
     *
     * Ensures the two node identifiers are not identical (a common netlist
     * error).
     *
     * @param nodeA Name of terminal A.
     * @param nodeB Name of terminal B.
     * @param lineNumber Line number for diagnostic messages.
     * @return True if nodes are valid (different), false otherwise.
     */
    bool validateNodes(const std::string& nodeA, const std::string& nodeB,
                       int lineNumber);

    /**
     * @brief Print a summary of element counts collected during parsing.
     *
     * Useful for quick diagnostics after parsing (how many resistors,
     * capacitors, sources, etc. were found).
     */
    void printElementCounts() const;

    /**
     * @brief Add very large-value resistors from each non-ground node to
     * ground.
     *
     * These "stability" resistors (very large ohms) help numeric solvers by
     * ensuring nodes without explicit DC paths are still connected to ground
     * for matrix conditioning. The resistor value is
     * `STABILITY_RESISTOR_VALUE`.
     */
    void addStabilityResistors();

    /**
     * @brief Transient analysis parameters (populated if `.TRAN` is present).
     *
     * `tranStep` is the time step and `tranStop` is the stop time for a
     * transient simulation. Values remain 0.0 if no `.TRAN` directive is seen.
     */
    double tranStep = 0.0;
    double tranStop = 0.0;

    /**
     * @brief Representation of a parsed subcircuit definition (.SUBCKT ...
     * .ENDS).
     *
     * The `body` field stores the raw, uppercased token-preserved lines that
     * appeared between the `.SUBCKT` header and the matching `.ENDS` directive.
     * `ports` stores the ordered port names declared on the `.SUBCKT` line.
     */
    struct SubcktDef
    {
        std::string name;               /**< Subcircuit name */
        std::vector<std::string> ports; /**< Ordered port names */
        std::vector<std::string> body;  /**< Raw uppercased body lines */
        int definitionLine = 0; /**< Line number where .SUBCKT appears */
    };

   private:
    /**
     * @brief Map of element name -> element pointer used to resolve references.
     *
     * The map is populated as elements are created and is later used to resolve
     * dependent-source controlling-element references and to support name
     * lookups.
     */
    std::map<std::string, std::shared_ptr<CircuitElement>> elementMap;

    /**
     * @brief Counters for parsed element types (incremented during parse()).
     */
    ElementCounts elementCounts;

    /**
     * @brief Value used for added stability resistors (very large ohms).
     */
    constexpr static double STABILITY_RESISTOR_VALUE = 1e12;

    /* ---------------------------------------------------------------------
     * Per-element parsing helpers
     *
     * The concrete parse helper functions accept a token vector and the
     * originating line number. They are implemented in the corresponding
     * Parser.cpp. These helpers create and append concrete element objects
     * to `circuitElements` and update `elementMap` and `nodes_group2`.
     * --------------------------------------------------------------------- */

    /**
     * @brief Parse a resistor line (R...).
     * @param tokens Tokenized line.
     * @param lineNumber Line number in the netlist.
     */
    void parseResistor(const std::vector<std::string>& tokens, int lineNumber);

    /**
     * @brief Parse an independent voltage source line (V...).
     * @param tokens Tokenized line.
     * @param lineNumber Line number in the netlist.
     */
    void parseVoltageSource(const std::vector<std::string>& tokens,
                            int lineNumber);

    /**
     * @brief Parse an independent current source line (I...).
     * @param tokens Tokenized line.
     * @param lineNumber Line number in the netlist.
     */
    void parseCurrentSource(const std::vector<std::string>& tokens,
                            int lineNumber);

    /**
     * @brief Parse a capacitor line (C...).
     * @param tokens Tokenized line.
     * @param lineNumber Line number in the netlist.
     */
    void parseCapacitor(const std::vector<std::string>& tokens, int lineNumber);

    /**
     * @brief Parse an inductor line (L...).
     * @param tokens Tokenized line.
     * @param lineNumber Line number in the netlist.
     */
    void parseInductor(const std::vector<std::string>& tokens, int lineNumber);

    /**
     * @brief Parse a dependent voltage source (VC...).
     * @param tokens Tokenized line.
     * @param lineNumber Line number in the netlist.
     */
    void parseDependentVoltageSource(const std::vector<std::string>& tokens,
                                     int lineNumber);

    /**
     * @brief Parse a dependent current source (IC...).
     * @param tokens Tokenized line.
     * @param lineNumber Line number in the netlist.
     */
    void parseDependentCurrentSource(const std::vector<std::string>& tokens,
                                     int lineNumber);

    // Collected subcircuit definitions discovered during parsing
    std::map<std::string, SubcktDef> subcktDefs;

    // Recursion/expansion guard default (configurable later)
    int subcktRecursionLimit = 32;
};
