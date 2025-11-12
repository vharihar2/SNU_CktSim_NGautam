#include "CircuitElement.hpp"
/**
 * @class DependentVoltageSource
 * @brief Represents a voltage source whose value depends on another circuit
 * variable.
 * @details Inherits from CircuitElement. Used for modeling controlled voltage
 * sources in circuit simulations.
 */
class DependentVoltageSource : public CircuitElement
{
   public:
    /**
     * @brief Constructs a DependentVoltageSource object.
     * @param name Name of the voltage source.
     * @param nodeA First node connection.
     * @param nodeB Second node connection.
     * @param value Control value for the source.
     */
    DependentVoltageSource(const std::string& name, const std::string& nodeA,
                           const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }

    /**
     * @brief Stamps the dependent voltage source into the MNA matrix and RHS
     * vector.
     * @param mna Modified Nodal Analysis matrix to be updated.
     * @param rhs Right-hand side vector to be updated.
     * @param indexMap Mapping from node names to matrix indices.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parses a DependentVoltageSource from a netlist line.
     * @param parser Reference to the Parser object.
     * @param tokens Tokenized netlist line.
     * @param lineNumber Line number in the netlist file.
     * @return Shared pointer to the created DependentVoltageSource object.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
};
