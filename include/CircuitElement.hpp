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
 *
 * @brief Contains the definition of the CircuitElement struct
 */

#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Parser.hpp"

/** @enum Component
 *
 * Specifies the type of element that are supported by CircuitElement
 * */
enum class ElementType
{
    V,  /**<  Voltage source */
    I,  /**<  Current source */
    R,  /**<  Resistor */
    Ic, /**<  Current controlled current source */
    Vc, /**<  Voltage controlled voltage source */
    C,  /**<  Capacitor */
    L,  /**< Inductor */

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
/** @enum Gruoup
 *
 * Specifies the group of element that are supported by CircuitElement
 * */
enum class Group
{
    G1 /**< Group 1: the currents for these elements need to be eliminated */,
    G2 /** Group 2: Elements not in G1. Other methods must be employed*/
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
/** @enum ControlVariable
 *
 * @brief Specifies the controlling variable for controlled sources
 * */
enum class ControlVariable
{
    none, /**<  No controlling variable */
    v,    /**<  Voltage */
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

/** @struct CircuitElement
 *
 * @brief Represents the properties of any element in the circuit
 * */

// struct CircuitElement
// {
//     std::string name;  /**< Name of the element*/
//     ElementType type;  /**< Specifies the type of the circuit element  */
//     std::string nodeA; /**< Starting node */
//     std::string nodeB; /**< Ending node */
//     Group group;       /**< Specifier if it belongs to  Group 1 or Group 2 */
//     double value; /**< Value of the element (or scale factor if it is
//     controlled
//                      source)*/
//     ControlVariable controlling_variable; /**< Only for controlled sources,
//                                      variable that the element depends on  */
//     std::shared_ptr<CircuitElement>
//         controlling_element; /**< Only for controlled sources, element whose
//                                 value that the current element depends on  */
//     bool processed;          /**< Flag value to know whether it is processed
//     */
// };

class CircuitElement
{
   protected:
    std::string name;
    std::string nodeA;
    std::string nodeB;
    Group group;
    double value;

    // needs to be removed after refactor
    ElementType type;

    // Dont know how i feel about this being common to all elements
    ControlVariable controlling_variable;
    std::shared_ptr<CircuitElement> controlling_element;
    bool processed;

   public:
    CircuitElement(const std::string& name, std::string& nodeA,
                   std::string& nodeB, double value)
        : name(name), nodeA(nodeA), nodeB(nodeB), value(value)
    {
    }

    virtual ~CircuitElement() = default;

    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) = 0;

    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
    bool isProcessed() const { return processed; }
    void setProcessed(bool val) { processed = val; }
};
