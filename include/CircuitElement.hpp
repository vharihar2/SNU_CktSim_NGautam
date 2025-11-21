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
 * @file CircuitElement.hpp
 * @brief Defines the CircuitElement base class and related enums
 *
 * This file contains the definition of the CircuitElement base class, which
 * represents a generic element in an electrical circuit. It also defines enums
 * for element type, grouping, and control variable, as well as utility
 * functions and the interface for parsing and dependency resolution.
 */

#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "../lib/external/Eigen/Dense"

// Forward Declaration
class Parser;

/**
 * @enum ElementType
 * @brief Specifies the type of circuit element supported by CircuitElement.
 */
enum class ElementType
{
    V,  /**< Voltage source */
    I,  /**< Current source */
    R,  /**< Resistor */
    Ic, /**< Current controlled current source */
    Vc, /**< Voltage controlled voltage source */
    C,  /**< Capacitor */
    L   /**< Inductor */
};

inline std::ostream& operator<<(std::ostream& os, ElementType et)
{
    switch (et) {
        case ElementType::V:
            os << "V";
            break;
        case ElementType::I:
            os << "I";
            break;
        case ElementType::R:
            os << "R";
            break;
        case ElementType::Ic:
            os << "IC";
            break;
        case ElementType::Vc:
            os << "VC";
            break;
        case ElementType::C:
            os << "C";
            break;
        case ElementType::L:
            os << "L";
            break;
        default:
            os << "UnknownElementType";
            break;
    }
    return os;
}
/**
 * @enum Group
 * @brief Specifies the group of circuit elements for matrix handling.
 *
 * Group 1: Elements whose currents need to be eliminated.
 * Group 2: Elements not in Group 1; other methods must be employed.
 */
enum class Group
{
    G1, /**< Group 1: the currents for these elements need to be eliminated */
    G2  /**< Group 2: Elements not in G1. Other methods must be employed */
};

inline std::ostream& operator<<(std::ostream& os, Group group)
{
    switch (group) {
        case Group::G1:
            os << "G1";
            break;
        case Group::G2:
            os << "G2";
            break;
        default:
            os << "UnknownGroup";
            break;
    }
    return os;
}
/**
 * @enum ControlVariable
 * @brief Specifies the controlling variable for controlled sources.
 */
enum class ControlVariable
{
    none, /**< No controlling variable */
    v,    /**< Voltage */
    i     /**< Current */
};

inline std::ostream& operator<<(std::ostream& os, ControlVariable cv)
{
    switch (cv) {
        case ControlVariable::none:
            os << "none";
            break;
        case ControlVariable::v:
            os << "v";
            break;
        case ControlVariable::i:
            os << "i";
            break;
        default:
            os << "UnknownControlVariable";
            break;
    }
    return os;
}

/**
 * @class CircuitElement
 * @brief Base class representing a generic element in an electrical circuit.
 *
 * This class provides the common interface and properties for all circuit
 * elements, such as resistors, capacitors, inductors, sources, and controlled
 * sources. Derived classes implement specific behavior for each element type.
 */
class CircuitElement
{
   protected:
    /**
     * @brief Name of the element (unique identifier)
     */
    std::string name;
    /**
     * @brief Name of the starting node
     */
    std::string nodeA;
    /**
     * @brief Name of the ending node
     */
    std::string nodeB;
    /**
     * @brief Group classification for the element (G1 or G2)
     */
    Group group = Group::G1;
    /**
     * @brief Value of the element (e.g., resistance, capacitance, etc.)
     */
    double value;
    /**
     * @brief Type of the element (enum)
     * @note This may be removed after refactor.
     */
    // TODO: Remove after full reafactor
    ElementType type;
    /**
     * @brief Controlling variable for controlled sources (none, voltage,
     * current)
     */
    ControlVariable controlling_variable = ControlVariable::none;
    /**
     * @brief Pointer to the controlling element (for controlled sources)
     */
    std::shared_ptr<CircuitElement> controlling_element = nullptr;
    /**
     * @brief Flag indicating whether the element has been processed
     */
    bool processed = false;

   public:
    /**
     * @brief Construct a CircuitElement
     * @param name Name of the element
     * @param nodeA Starting node name
     * @param nodeB Ending node name
     * @param value Value of the element
     */
    CircuitElement(const std::string& name, const std::string& nodeA,
                   const std::string& nodeB, double value)
        : name(name), nodeA(nodeA), nodeB(nodeB), value(value)
    {
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~CircuitElement() = default;

    /**
     * @brief Stamps the element's contribution into the MNA matrix and RHS
     * vector
     * @param mna Modified Nodal Analysis matrix
     * @param rhs Right-hand side vector
     * @param indexMap Map from node/element names to matrix indices
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) = 0;

    // NOTE: --- Transient / time-domain hooks --
    virtual void computeCompanion([[maybe_unused]] double h) {}
    // Stamp the transient companion contributions for this element into `mna`
    // and `rhs`. Default implementation forwards to the DC `stamp` behavior so
    // existing elements compile.
    virtual void stampTransient(std::vector<std::vector<double>>& mna,
                                std::vector<double>& rhs,
                                std::map<std::string, int>& indexMap)
    {
        // By default, use the DC stamp (useful until individual elements
        // override this).
        stamp(mna, rhs, indexMap);
    }

    // Update the element's internal state (e.g., capacitor voltage, inductor
    // current) from the solved solution vector `x`. `indexMap` maps variable
    // names to indices in `x`. Default: no-op.
    virtual void updateStateFromSolution(
        const Eigen::Ref<const Eigen::VectorXd>& x,
        const std::map<std::string, int>& indexMap)
    {
    }
    /**
     * @brief Parses a circuit element from tokens
     * @param parser Reference to the parser
     * @param tokens Tokenized line from netlist
     * @param lineNumber Line number in netlist
     * @return Shared pointer to created CircuitElement
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);

    /**
     * @brief Resolves dependencies for controlled sources
     * @param circuitElements Vector of all circuit elements
     * @param elementMap Map from element names to CircuitElement pointers
     * @param nodes_group2 Set of node names and group_2 element names
     * @return Number of errors encountered
     */
    static int resolveDependencies(
        std::vector<std::shared_ptr<CircuitElement>>& circuitElements,
        std::map<std::string, std::shared_ptr<CircuitElement>>& elementMap,
        std::set<std::string>& nodes_group2);

    /**
     * @brief Checks if the element has been processed
     * @return True if processed, false otherwise
     */
    bool isProcessed() const { return processed; }
    /**
     * @brief Sets the processed flag
     * @param val New value for processed flag
     */
    void setProcessed(bool val) { processed = val; }

    /**
     * @brief Gets the element name
     * @return Name of the element
     */
    std::string getName() const { return name; }
    /**
     * @brief Sets the element name
     * @param val New name
     */
    void setName(const std::string& val) { name = val; }

    /**
     * @brief Gets the starting node name
     * @return Name of nodeA
     */
    std::string getNodeA() const { return nodeA; }
    /**
     * @brief Gets the ending node name
     * @return Name of nodeB
     */
    std::string getNodeB() const { return nodeB; }

    /**
     * @brief Gets the group classification
     * @return Group enum value
     */
    Group getGroup() const { return group; }
};
