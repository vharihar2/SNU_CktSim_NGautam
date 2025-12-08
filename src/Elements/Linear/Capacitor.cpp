/* Copyright (c) 2022, Shiv Nadar University, Delhi NCR, India. All Rights
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
 * @file Capacitor.cpp
 * @brief Implementation of the Capacitor class declared in Capacitor.hpp.
 *
 * This file provides the concrete implementations for the Capacitor element's
 * methods: DC stamp (no-op), parsing factory, trapezoidal companion
 * computation, transient stamping (Norton equivalent), and state update from
 * the solver's solution vector.
 */

#include "Capacitor.hpp"

#include <memory>

#include "CircuitElement.hpp"
#include "Parser.hpp"

void Capacitor::stamp(std::vector<std::vector<double>>& mna,
                      std::vector<double>& rhs,
                      std::map<std::string, int>& indexMap)
{
    // Open circuit in DC; nothing to stamp. Mark parameters as used to avoid
    // unused-parameter warnings.
    (void)mna;
    (void)rhs;
    (void)indexMap;
    return;
}

std::shared_ptr<CircuitElement> Capacitor::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Valid token counts: 4 or 5 (for optional group)
    if (!parser.validateTokens(tokens, 4, lineNumber) &&
        !parser.validateTokens(tokens, 5, lineNumber)) {
        std::cerr << "Error: Invalid capacitor definition at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Capacitor nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for capacitor value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element =
        std::make_shared<Capacitor>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::C;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

// Compute trapezoidal-rule companion parameters for timestep h.
// Geq = 2 * C / h
// Ieq = Geq * v_prev - i_prev
void Capacitor::computeCompanion(double h)
{
    // Protect against non-positive timestep.
    if (h <= 0.0) {
        Geq = 0.0;
        Ieq = 0.0;
        return;
    }

    // Compute Geq and Ieq using trapezoidal companion.
    Geq = 2.0 * value / h;
    Ieq = Geq * v_prev - i_prev;
}

void Capacitor::stampTransient(std::vector<std::vector<double>>& mna,
                               std::vector<double>& rhs,
                               std::map<std::string, int>& indexMap)
{
    // Stamp Norton companion (Geq between nodes, Ieq injected). Assumes
    // computeCompanion called.

    // Return index for node or -1 for ground/missing
    auto idx = [&](const std::string& node) -> int {
        if (node == "0") return -1;
        auto it = indexMap.find(node);
        return (it == indexMap.end()) ? -1 : it->second;
    };

    int vplus = idx(nodeA);
    int vminus = idx(nodeB);

    // Stamp conductance Geq into MNA, handling ground.
    if (vplus == -1 && vminus == -1) {
        // nothing to stamp
    } else if (vplus == -1) {
        mna[vminus][vminus] += Geq;
    } else if (vminus == -1) {
        mna[vplus][vplus] += Geq;
    } else {
        mna[vplus][vplus] += Geq;
        mna[vplus][vminus] -= Geq;
        mna[vminus][vplus] -= Geq;
        mna[vminus][vminus] += Geq;
    }

    // Inject companion current Ieq (from nodeA -> nodeB) into RHS.
    if (vplus != -1) {
        rhs[vplus] -= Ieq;
    }
    if (vminus != -1) {
        rhs[vminus] += Ieq;
    }
}

void Capacitor::updateStateFromSolution(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    const std::map<std::string, int>& indexMap)
{
    // Read node voltages (ground/missing -> 0)
    double vplus = 0.0;
    double vminus = 0.0;

    // Handle ground/missing nodes
    if (nodeA != "0") {
        auto it = indexMap.find(nodeA);
        if (it != indexMap.end()) vplus = x[it->second];
    }

    if (nodeB != "0") {
        auto it = indexMap.find(nodeB);
        if (it != indexMap.end()) vminus = x[it->second];
    }

    // Update state: v_prev and i_prev (i_{n+1} = Geq * v_{n+1} - Ieq)
    double u_n1 = vplus - vminus;
    v_prev = u_n1;
    i_prev = Geq * u_n1 - Ieq;
}
