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
 * @brief Enumerates supported circuit element categories.
 *
 * Each enumerator identifies a concrete element class used by the parser and
 * solver. This enum is primarily used for diagnostics and to simplify simple
 * switch-based behaviour in legacy code; it may be removed as the codebase is
 * refactored to use polymorphism or type traits.
 */
enum class ElementType
{
    V,  /**< Independent voltage source */
    I,  /**< Independent current source */
    R,  /**< Resistor */
    Ic, /**< Current-controlled current source (CCCS) */
    Vc, /**< Voltage-controlled voltage source (VCVS) */
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
 * @brief Classification used during Modified Nodal Analysis (MNA) assembly.
 *
 * Elements are classified into groups to guide how they are represented in the
 * MNA system:
 *  - G1: Elements whose branch currents are eliminated from the primary
 *        nodal system (for example, using current elimination or passive
 *        injection techniques).
 *  - G2: Elements that typically require auxiliary unknowns (e.g. branch
 *        currents for voltage sources or controlled sources) and therefore
 *        a different stamping strategy.
 *
 * This classification affects both DC and transient stamping paths.
 */
enum class Group
{
    G1, /**< Group 1: branch currents eliminated during assembly */
    G2  /**< Group 2: auxiliary unknowns (explicit branch current variables) */
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
 * @brief Type of the controlling quantity used by dependent sources.
 *
 * Controlled (dependent) sources refer to another circuit variable when
 * computing their output. This enum identifies the controlling quantity:
 *  - none: not a controlled source / no controlling variable set
 *  - v:    the source is controlled by a voltage elsewhere in the circuit
 *  - i:    the source is controlled by a current elsewhere in the circuit
 */
enum class ControlVariable
{
    none, /**< Not a controlled source */
    v,    /**< Voltage-controlled */
    i     /**< Current-controlled */
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

    /**
     * @brief Compute per-timestep companion parameters for transient solves.
     *
     * Linear elements (C, L, etc.) can compute a time-step dependent companion
     * model (e.g. Norton/Thévenin equivalents using the trapezoidal rule or
     * backward-Euler). The solver calls this method once per time-step to allow
     * the element to prepare coefficients (conductances, equivalent currents,
     * etc.) used during transient assembly.
     *
     * Default implementation is a no-op for elements that have no transient
     * behaviour.
     *
     * @param h Time-step size (seconds). Implementations should guard against
     *          non-positive values.
     */
    virtual void computeCompanion([[maybe_unused]] double h) {}

    /**
     * @brief Per-Newton-iteration companion update for nonlinear TR solves.
     *
     * For nonlinear elements participating in a Newton solve per time-step,
     * this hook allows updating companion parameters based on the current
     * Newton iterate `xk`. The default forwards to `computeCompanion(h)` to
     * preserve backwards compatibility with linear elements.
     *
     * @param h Current time-step size.
     * @param xk Current Newton iterate (solution vector).
     * @param indexMap Map from node/element names to indices in `xk`.
     */
    virtual void computeCompanionIter(
        [[maybe_unused]] double h,
        [[maybe_unused]] const Eigen::Ref<const Eigen::VectorXd>& xk,
        [[maybe_unused]] const std::map<std::string, int>& indexMap)
    {
        computeCompanion(h);
    }

    /**
     * @brief Indicates whether the element is nonlinear.
     *
     * Nonlinear elements should override this to return true so the transient
     * solver can decide whether to invoke iterative companion update hooks.
     *
     * @return true if element is nonlinear, false otherwise (default).
     */
    virtual bool isNonlinear() const { return false; }

    /**
     * @brief Stamp transient companion contributions for this element.
     *
     * Elements that implement a transient companion model should override this
     * method to insert per-time-step conductances / equivalent sources into the
     * provided `mna` and `rhs`. The default implementation simply calls the DC
     * `stamp(...)` method so that elements without transient behavior remain
     * compatible.
     *
     * @param mna Modified Nodal Analysis matrix (mutated in-place).
     * @param rhs Right-hand side vector (mutated in-place).
     * @param indexMap Map from node/element names to indices in `mna`/`rhs`.
     */
    virtual void stampTransient(std::vector<std::vector<double>>& mna,
                                std::vector<double>& rhs,
                                std::map<std::string, int>& indexMap)
    {
        // Default: fall back to DC stamp.
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

    // Diagnostic hook: elements can emit a compact diagnostic snapshot for a
    // given solver iterate `xk`. Default implementation is a no-op.
    virtual void dumpDiagnostics(
        std::ostream& /*os*/, const Eigen::Ref<const Eigen::VectorXd>& /*xk*/,
        const std::map<std::string, int>& /*indexMap*/) const
    {
    }

    // --- Checkpoint / restore API ---
    // Elements should return a compact vector of doubles that represent the
    // minimal internal state required to restore the element to a previous
    // timestep. The solver will call `snapshotState()` before a step and
    // `restoreState()` if the step needs to be retried.
    virtual std::vector<double> snapshotState() const { return {}; }
    virtual void restoreState(const std::vector<double>& /*data*/) {}
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
