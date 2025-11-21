#include <string>

#include "CircuitElement.hpp"

/**
 * @file Capacitor.hpp
 * @brief Defines the Capacitor class for circuit simulation.
 *
 * This file contains the definition of the Capacitor class, which models a
 * capacitor element in an electrical circuit. It inherits from CircuitElement
 * and implements the required interface for stamping and parsing.
 */

/**
 * @class Capacitor
 * @brief Represents a capacitor element in the circuit.
 *
 * The Capacitor class provides methods for stamping its contribution into the
 * MNA matrix and for parsing a capacitor from a netlist. Inherits from
 * CircuitElement.
 */
class Capacitor : public CircuitElement
{
   private:
    // Voltage across capacitor at previous timestep (u_n)
    double v_prev = 0.0;
    // Capacitor current at previous timestep (i_n) — optional, useful for some
    // updates
    double i_prev = 0.0;
    // Companion model parameters for current time-step (computed by
    // computeCompanion)
    double Geq = 0.0;  // equivalent conductance = 2*C / h
    double Ieq = 0.0;  // equivalent current source

   public:
    /**
     * @brief Constructs a Capacitor element.
     * @param name Name of the capacitor element.
     * @param nodeA Starting node name.
     * @param nodeB Ending node name.
     * @param value Capacitance value.
     */
    Capacitor(const std::string& name, const std::string& nodeA,
              const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }
    /**
     * @brief Stamps the capacitor's contribution into the MNA matrix and RHS
     * vector.
     * @param mna Modified Nodal Analysis matrix.
     * @param rhs Right-hand side vector.
     * @param indexMap Map from node/element names to matrix indices.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parses a capacitor element from netlist tokens.
     * @param parser Reference to the parser.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     * @return Shared pointer to the created Capacitor element.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);

    void computeCompanion(double h) override;

    void stampTransient(std::vector<std::vector<double>>& mna,
                        std::vector<double>& rhs,
                        std::map<std::string, int>& indexMap) override;

    void updateStateFromSolution(
        const Eigen::Ref<const Eigen::VectorXd>& x,
        const std::map<std::string, int>& indexMap) override;
};
