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
#pragma once

/**
 * @file Resistor.hpp
 * @brief Declaration of the Resistor element used in circuit simulation.
 *
 * This header declares the `Resistor` concrete element class which models an
 * ideal two-terminal resistor for use by the simulator. The class derives from
 * `CircuitElement` and provides:
 *
 *  - A constructor to create a resistor instance programmatically.
 *  - A static `parse()` helper used by `Parser` to construct a resistor from
 *    tokenized netlist input.
 *  - A `stamp()` method which inserts the resistor's contributions into the
 *    Modified Nodal Analysis (MNA) matrix and RHS vector.
 *
 * Notes on stamping:
 *  - The simulator supports two element groups:
 *      - Group 1 (G1): standard passive elements represented only via node
 *        equations (no extra unknowns).
 *      - Group 2 (G2): elements whose currents are explicit unknowns in the
 *        system (these require additional rows/columns in the MNA matrix).
 *  - When a resistor is placed in G2 its current becomes an unknown; the
 *    `stamp()` implementation below handles both G1 and G2 cases and the
 *    special handling when one terminal is ground ('0').
 *
 * Usage example:
 * @code
 * // Programmatic construction
 * Resistor r(\"R1\", \"1\", \"0\", 1000.0); // 1k between node 1 and ground
 *
 * // Parsing (inside Parser.cpp)
 * auto elem = Resistor::parse(parser, tokens, lineNumber);
 * if (elem) {  // returns shared_ptr<CircuitElement>
 *   parser.circuitElements.push_back(elem);
 * }
 * @endcode
 */

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "CircuitElement.hpp"
#include "Parser.hpp"

/**
 * @class Resistor
 * @brief Concrete resistor element implementing a two-terminal resistor.
 *
 * The `Resistor` class encapsulates the electrical behavior of an ideal linear
 * resistor. It implements the `stamp()` method which inserts the appropriate
 * conductance entries into the MNA matrix depending on whether the resistor
 * is modeled as a pure nodal element (group 1) or with an explicit current
 * unknown (group 2).
 *
 * Parsing:
 *  - The static `parse()` method expects tokenized, uppercased netlist tokens
 *    arranged in the SPICE convention:
 *
 *      R<name> <nodeA> <nodeB> <value> [optional_group_token]
 *
 *  - `parse()` uses `Parser::validateTokens`, `Parser::validateNodes` and
 *    `Parser::parseValue` to validate and read the numeric resistance.
 *
 * Stamping details:
 *  - For group 1 (G1), the resistor contributes conductances between node
 *    indices in the classical 2x2 stamp (or a single diagonal entry if one
 *    end is ground).
 *  - For group 2 (G2), the resistor's current is an explicit unknown and the
 *    stamp builds the couplings between node voltages and the element current
 *    variable and fills the element-local current-voltage relation row.
 */
class Resistor : public CircuitElement
{
   public:
    /**
     * @brief Construct a resistor element.
     *
     * @param name Unique element name (e.g., "R1").
     * @param nodeA Positive terminal node name.
     * @param nodeB Negative terminal node name.
     * @param value Resistance value in ohms.
     */
    Resistor(const std::string& name, const std::string& nodeA,
             const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
        type = ElementType::R;
    }

    /**
     * @brief Stamp the resistor into the MNA matrix and RHS vector.
     *
     * This routine modifies `mna` and `rhs` according to the element's group:
     *  - Group 1: add conductance entries between node indices, treating
     *    ground ('0') specially by emitting single diagonal entries.
     *  - Group 2: treat the resistor's current as an additional unknown with
     *    name `this->name` (index looked up via `indexMap`). The stamp writes
     *    KCL couplings between node voltages and the element current and the
     *    resistor's I-V relation row (Ohm's law) into the MNA matrix. The
     *    method assumes `indexMap` contains indices for node names and, for
     *    G2 elements, an index keyed by the element name.
     *
     * @param mna Reference to the square MNA matrix (modified in-place).
     * @param rhs Reference to the RHS vector (modified if needed).
     * @param indexMap Mapping from node/element names to matrix indices.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parse a resistor from tokenized netlist input.
     *
     * Expected token forms:
     *  - Standard: Rname nodeA nodeB value
     *  - Optional: Rname nodeA nodeB value <group-indicator>
     *
     * The function performs:
     *  - Token count validation via `parser.validateTokens`.
     *  - Node name validation via `parser.validateNodes`.
     *  - Numeric value parsing via `parser.parseValue`.
     *
     * On success returns a `shared_ptr` owning the constructed `Resistor`
     * instance. On failure it prints diagnostic messages to stderr and
     * returns `nullptr`.
     *
     * @param parser Reference to the calling `Parser` instance (used for
     *               validation and helpers).
     * @param tokens Tokenized, uppercased tokens for the current line.
     * @param lineNumber Original line number in the netlist (for diagnostics).
     * @return shared_ptr<CircuitElement> owning the created resistor or
     *         `nullptr` on error.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
};
