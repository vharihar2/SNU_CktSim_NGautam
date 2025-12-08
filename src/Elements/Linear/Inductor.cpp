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
 * @file Inductor.cpp
 * @brief Implementation of the `Inductor` class declared in Inductor.hpp.
 *
 * Provides parsing, DC stamping, transient companion computation and stamping,
 * and state-update logic for the linear inductor element. Comments are kept
 * short and focused on the implementation.
 */

#include "Inductor.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Parser.hpp"

std::shared_ptr<CircuitElement> Inductor::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Expect: Lname nodeA nodeB value
    if (!parser.validateTokens(tokens, 4, lineNumber)) {
        std::cerr << "Error: Invalid inductor definition at line " << lineNumber
                  << std::endl;
        return nullptr;
    }

    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Inductor nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for inductor value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element =
        std::make_shared<Inductor>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::L;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

void Inductor::stamp(std::vector<std::vector<double>>& mna,
                     std::vector<double>& rhs,
                     std::map<std::string, int>& indexMap)
{
    // DC stamp: introduce branch-current unknown coupling for Group-2
    // inductors.
    if (nodeA == "0") {
        int vminus = indexMap[nodeB];
        int i = indexMap[name];
        mna[vminus][i] += 1.0;
        mna[i][vminus] += 1.0;
    } else if (nodeB == "0") {
        int vplus = indexMap[nodeA];
        int i = indexMap[name];
        mna[vplus][i] += 1.0;
        mna[i][vplus] += 1.0;
    } else {
        int vplus = indexMap[nodeA];
        int vminus = indexMap[nodeB];
        int i = indexMap[name];
        mna[vplus][i] += 1.0;
        mna[vminus][i] += -1.0;
        mna[i][vplus] += 1.0;
        mna[i][vminus] += -1.0;
    }
}

// Compute trapezoidal-rule companion parameters for timestep h.
// Geq = h / (2 * L)
// Ieq = i_prev + Geq * u_prev
void Inductor::computeCompanion(double h)
{
    if (h <= 0.0) {
        Geq = 0.0;
        Ieq = 0.0;
        return;
    }

    // 'value' stores inductance L
    Geq = h / (2.0 * value);
    Ieq = i_prev + Geq * u_prev;
}

void Inductor::stampTransient(std::vector<std::vector<double>>& mna,
                              std::vector<double>& rhs,
                              std::map<std::string, int>& indexMap)
{
    // If Geq is zero (e.g., h<=0) skip companion stamping.
    if (Geq == 0.0) return;

    // Series-equivalent resistance for the branch equation
    double Rseries = 1.0 / Geq;

    // Branch-current index if present
    int i = -1;
    auto it_i = indexMap.find(name);
    if (it_i != indexMap.end()) i = it_i->second;

    // Node indices (-1 => ground/missing)
    int vplus = -1;
    int vminus = -1;
    if (nodeA != "0") {
        auto it = indexMap.find(nodeA);
        if (it != indexMap.end()) vplus = it->second;
    }
    if (nodeB != "0") {
        auto it = indexMap.find(nodeB);
        if (it != indexMap.end()) vminus = it->second;
    }

    // If branch-current unknown exists, stamp series branch form
    if (i != -1) {
        if (vplus != -1) {
            mna[vplus][i] += 1.0;
            mna[i][vplus] += 1.0;
        }
        if (vminus != -1) {
            mna[vminus][i] += -1.0;
            mna[i][vminus] += -1.0;
        }

        // Branch diagonal (series R) and RHS branch injection
        mna[i][i] += -Rseries;
        double rhs_branch = -Ieq / Geq;
        rhs[i] += rhs_branch;
        return;
    }

    // Fallback: stamp Norton equivalent between nodes
    auto idx = [&](const std::string& node) -> int {
        if (node == "0") return -1;
        auto it = indexMap.find(node);
        return (it == indexMap.end()) ? -1 : it->second;
    };

    int na = idx(nodeA);
    int nb = idx(nodeB);
    if (na == -1 && nb == -1) {
        return;
    } else if (na == -1) {
        mna[nb][nb] += Geq;
        rhs[nb] += Ieq;
    } else if (nb == -1) {
        mna[na][na] += Geq;
        rhs[na] -= Ieq;
    } else {
        mna[na][na] += Geq;
        mna[na][nb] -= Geq;
        mna[nb][na] -= Geq;
        mna[nb][nb] += Geq;
        rhs[na] -= Ieq;
        rhs[nb] += Ieq;
    }
}

void Inductor::updateStateFromSolution(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    const std::map<std::string, int>& indexMap)
{
    // Read node voltages (ground/missing -> 0)
    double vplus = 0.0;
    double vminus = 0.0;

    if (nodeA != "0") {
        auto it = indexMap.find(nodeA);
        if (it != indexMap.end()) vplus = x[it->second];
    }
    if (nodeB != "0") {
        auto it = indexMap.find(nodeB);
        if (it != indexMap.end()) vminus = x[it->second];
    }

    double u_n1 = vplus - vminus;
    u_prev = u_n1;

    // Prefer reading branch current if available
    auto it_i = indexMap.find(name);
    if (it_i != indexMap.end()) {
        int i_index = it_i->second;
        i_prev = x[i_index];
    } else {
        // Fallback: compute from companion relation
        i_prev = Geq * u_n1 + Ieq;
    }
}
