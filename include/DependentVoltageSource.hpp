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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "CircuitElement.hpp"

// Forward declaration used by the parse factory.
class Parser;

/**
 * @file DependentVoltageSource.hpp
 * @brief Declaration of the DependentVoltageSource (controlled voltage source).
 *
 * This header declares the `DependentVoltageSource` class which models an ideal
 * controlled voltage source. The source value is controlled by either a voltage
 * or a current elsewhere in the circuit. The element supports both voltage-
 * controlled voltage sources (VCVS) and current-controlled voltage sources
 * (CCVS) depending on the `controlling_variable` set by the parser.
 *
 * Controlled sources belong to Group G2 for MNA assembly and therefore
 * typically introduce an extra current unknown (node for the source) and
 * appropriate matrix stamps. The `parse` factory validates tokens and
 * configures the controlling variable and (later) the controlling element
 * pointer from the parser/resolution step.
 *
 * See `CircuitElement.hpp` for the base-class interface and simulation hooks.
 */

/**
 * @class DependentVoltageSource
 * @brief Controlled (dependent) voltage source element.
 *
 * The `DependentVoltageSource` models an ideal voltage source whose amplitude
 * is a scalar multiple (`value`) of a controlling variable:
 *
 *   - For VCVS: v_source = value * v_control
 *   - For CCVS: v_source = value * i_control
 *
 * The parser must supply the controlling variable type ("V" or "I") and the
 * name of the controlling element. Cascading controlled sources (a controlled
 * source controlled by another controlled source) are not permitted by the
 * parser and will be rejected.
 */
class DependentVoltageSource : public CircuitElement
{
   public:
    /**
     * @brief Construct a new DependentVoltageSource.
     *
     * @param name Element identifier (e.g., "VC1").
     * @param nodeA Positive/source node name (netlist token 1).
     * @param nodeB Negative/sink node name (netlist token 2).
     * @param value Scalar gain for the controlled source (netlist token 3).
     */
    DependentVoltageSource(const std::string& name, const std::string& nodeA,
                           const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }

    /**
     * @brief Stamp the dependent voltage source into the MNA matrix and RHS.
     *
     * For Group G2 controlled voltage sources the stamping introduces the
     * additional current unknown associated with the source and adds the
     * appropriate rows/columns linking the source nodes and the controlling
     * element (depending on whether the control is a voltage or current).
     *
     * Implementation notes / caller responsibilities:
     *  - `indexMap` must contain indices for the capacitor nodes and for the
     *    additional unknown (typically mapped using the element name).
     *  - `controlling_element` should be resolved (non-null) when the stamp
     *    requires access to the controlling element's node indices.
     *
     * @param mna Modified Nodal Analysis matrix (mutated in-place).
     * @param rhs Right-hand side vector (mutated in-place).
     * @param indexMap Map from node/element names to indices in `mna`/`rhs`.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parse a dependent voltage source from tokenized netlist input.
     *
     * Expected token format:
     *   Vcname nodeA nodeB value <V|I> controllingElementName
     *
     * The parser validates token count and node names, parses the numeric
     * value, and records the requested controlling variable. The returned
     * pointer will have `type` set to `ElementType::Vc` and `group` set to
     * `Group::G2`. The actual pointer to the controlling element is resolved
     * later by `CircuitElement::resolveDependencies`.
     *
     * On error the function returns `nullptr` and emits diagnostics to stderr.
     *
     * @param parser Parser helper used for token validation and numeric
     * parsing.
     * @param tokens Tokenized netlist line.
     * @param lineNumber Source line number for diagnostics.
     * @return std::shared_ptr<CircuitElement> Created DependentVoltageSource or
     * nullptr on error.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
};
