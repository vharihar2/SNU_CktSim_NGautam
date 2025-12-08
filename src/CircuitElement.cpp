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
 * @file CircuitElement.cpp
 * @brief Implementation helpers for circuit element dependency resolution.
 *
 * This file implements utility routines related to `CircuitElement` objects
 * used by the parser and solver. The primary function exported here,
 * `CircuitElement::resolveDependencies`, walks parsed elements and resolves
 * references used by dependent sources (e.g., IC/VC elements that reference
 * other elements by name). It also enforces that elements whose current is
 * referenced are placed into Group::G2 and reports diagnostics for missing
 * references.
 *
 * Keep implementation comments concise — the public API and behavior are
 * documented in the header (`CircuitElement.hpp`).
 */

#include "CircuitElement.hpp"

#include <map>
#include <set>
#include <string>

/**
 * @brief Resolve controlling-element references for dependent sources.
 *
 * The parser initially constructs dependent sources with a placeholder
 * `controlling_element` pointer whose `name` field contains the name of the
 * referenced element. This function attempts to replace those placeholders
 * with actual shared_ptrs obtained from `elementMap`. If the controlling
 * variable is a current (`ControlVariable::i`), the referenced element must
 * belong to `Group::G2` (its current appears as an unknown in MNA); when it
 * does not, we promote the referenced element to G2 and add its name to
 * `nodes_group2`.
 *
 * Any unresolved references are reported to stderr and counted as errors.
 *
 * @param circuitElements Vector of parsed element shared pointers (may include
 * nullptrs).
 * @param elementMap Map from element name -> shared_ptr<CircuitElement> used
 * for resolution.
 * @param nodes_group2 Output set of node/group-2 names; updated when promotions
 * happen.
 * @return Number of resolution errors encountered.
 */
int CircuitElement::resolveDependencies(
    std::vector<std::shared_ptr<CircuitElement>> &circuitElements,
    std::map<std::string, std::shared_ptr<CircuitElement>> &elementMap,
    std::set<std::string> &nodes_group2)
{
    int erroCount = 0;

    // Iterate through elements and resolve any controlling_element
    // placeholders.
    for (auto &elem : circuitElements) {
        if (elem->controlling_variable != ControlVariable::none &&
            elem->controlling_element) {
            // Look up the referenced element by name in the map produced during
            // parsing.
            auto it = elementMap.find(elem->controlling_element->name);
            if (it != elementMap.end()) {
                // Replace the placeholder with the real shared_ptr from the
                // map.
                elem->controlling_element = it->second;

                // If the dependent source references the current of another
                // element, that referenced element must be in Group::G2 so its
                // current is an explicit unknown in MNA. Promote and record if
                // necessary.
                if (elem->controlling_variable == ControlVariable::i &&
                    elem->controlling_element->group != Group::G2) {
                    std::cerr << "Warning: Referenced element "
                              << elem->controlling_element->name
                              << " must be in group 2 as its current variable "
                                 "is required by "
                              << elem->name << std::endl;
                    elem->controlling_element->group = Group::G2;
                    nodes_group2.insert(elem->controlling_element->name);
                }
            } else {
                // Missing referenced element: emit an error and increment
                // count.
                std::cerr << "Error: Referenced element "
                          << elem->controlling_element->name
                          << " (referenced by " << elem->name
                          << ") is not present in the netlist" << std::endl;
                erroCount++;
            }
        }
    }
    return erroCount;
}
