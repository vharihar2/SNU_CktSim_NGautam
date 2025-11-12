#include <memory>
#include <string>
#include <vector>

#include "CircuitElement.hpp"
#include "Parser.hpp"

/**
 * @file Resistor.hpp
 * @brief Defines the Resistor class for circuit simulation.
 *
 * This file contains the definition of the Resistor class, which models a
 * resistor element in an electrical circuit. It inherits from CircuitElement
 * and implements the required interface for stamping and parsing.
 */

/**
 * @class Resistor
 * @brief Represents a resistor element in the circuit.
 *
 * The Resistor class provides methods for stamping its contribution into the
 * MNA matrix and for parsing a resistor from a netlist.
 */
class Resistor : public CircuitElement
{
   public:
    /**
     * @brief Constructs a Resistor element.
     * @param name Name of the resistor element.
     * @param nodeA Starting node name.
     * @param nodeB Ending node name.
     * @param value Resistance value.
     */
    Resistor(const std::string& name, const std::string& nodeA,
             const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
        type = ElementType::R;
    }

    /**
     * @brief Stamps the resistor's contribution into the MNA matrix and RHS
     * vector.
     * @param mna Modified Nodal Analysis matrix.
     * @param rhs Right-hand side vector.
     * @param indexMap Map from node/element names to matrix indices.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parses a resistor element from netlist tokens.
     * @param parser Reference to the parser.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     * @return Shared pointer to the created Resistor element.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
};
