#include "CircuitElement.hpp"

/**
 * @file CurrentSource.hpp
 * @brief Defines the CurrentSource class for circuit simulation.
 *
 * This file contains the definition of the CurrentSource class, which models a
 * current source element in an electrical circuit. It inherits from
 * CircuitElement and implements the required interface for stamping and
 * parsing.
 */

/**
 * @class CurrentSource
 * @brief Represents a current source element in the circuit.
 *
 * The CurrentSource class provides methods for stamping its contribution into
 * the MNA matrix and for parsing a current source from a netlist. Inherits from
 * CircuitElement.
 */
class CurrentSource : public CircuitElement
{
   public:
    /**
     * @brief Constructs a CurrentSource element.
     * @param name Name of the current source element.
     * @param nodeA Starting node name.
     * @param nodeB Ending node name.
     * @param value Current value.
     */
    CurrentSource(const std::string& name, const std::string& nodeA,
                  const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }
    /**
     * @brief Stamps the current source's contribution into the MNA matrix and
     * RHS vector.
     * @param mna Modified Nodal Analysis matrix.
     * @param rhs Right-hand side vector.
     * @param indexMap Map from node/element names to matrix indices.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parses a current source element from netlist tokens.
     * @param parser Reference to the parser.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     * @return Shared pointer to the created CurrentSource element.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
};
