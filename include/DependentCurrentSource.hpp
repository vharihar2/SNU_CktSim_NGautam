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

// Forward declaration for parser used by the parse factory.
class Parser;

/**
 * @file DependentCurrentSource.hpp
 * @brief Declaration of the DependentCurrentSource element.
 *
 * This header declares the `DependentCurrentSource` class which models an ideal
 * dependent (controlled) current source. The source value is a scalar multiple
 * of a controlling circuit variable: either a controlling voltage (V) or a
 * controlling current (I). The element supports both voltage-controlled
 * current source (VCCS) and current-controlled current source (CCCS)
 * semantics depending on the parser-specified `controlling_variable`.
 *
 * The class derives from `CircuitElement` and provides a DC stamping routine
 * and a parsing factory used by the netlist parser. Controlled sources are
 * typically classified as Group G2 devices in this simulator and may require
 * access to the controlling element during stamping; the parser sets the
 * controlling-element name and the dependency resolution step fills the
 * `controlling_element` pointer.
 *
 * See `CircuitElement.hpp` for the base-class API and simulation hooks.
 */
class DependentCurrentSource : public CircuitElement
{
   public:
    /**
     * @brief Construct a new DependentCurrentSource.
     *
     * @param name Element identifier (e.g., "G1").
     * @param nodeA Positive/source node name (current leaves this node).
     * @param nodeB Negative/sink node name (current enters this node).
     * @param value Scalar gain for the controlled source (unit depends on
     * control).
     */
    DependentCurrentSource(const std::string& name, const std::string& nodeA,
                           const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }

    /**
     * @brief Stamp the dependent current source into the MNA system.
     *
     * Behavior depends on whether the source is voltage-controlled (VCCS) or
     * current-controlled (CCCS). For DC assembly the source will inject
     * currents into the RHS and, when required by the formulation, may add
     * matrix terms that couple to the controlling element. The implementation
     * expects `indexMap` to contain indices for the element's node names and
     * for any additional unknowns (if the device formulation requires them). If
     * the stamping depends on the controlling element, `controlling_element`
     * must have been resolved prior to calling this method.
     *
     * @param mna Modified Nodal Analysis matrix (mutated in-place).
     * @param rhs Right-hand side vector (mutated in-place).
     * @param indexMap Map from node/element names to indices in `mna`/`rhs`.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parse a dependent current source from tokenized netlist input.
     *
     * Expected token format:
     *   Gname nodeA nodeB value <V|I> controllingElementName
     *
     * The parser validates token count and node names, parses the numeric
     * coefficient `value`, and records the requested controlling variable.
     * The returned pointer will have `type` set to `ElementType::Ic` (or the
     * project's chosen enum) and `group` typically set to `Group::G2`. The
     * actual pointer to the controlling element is resolved later by
     * `CircuitElement::resolveDependencies`.
     *
     * On error the function returns `nullptr` and emits diagnostics to stderr.
     *
     * @param parser Parser helper used for token validation and numeric
     * parsing.
     * @param tokens Tokenized netlist line.
     * @param lineNumber Line number in the netlist (for diagnostics).
     * @return std::shared_ptr<CircuitElement> Created element or nullptr on
     * error.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
};
