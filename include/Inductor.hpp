#include "CircuitElement.hpp"

/**
 * @file Inductor.hpp
 * @brief Defines the Inductor class for circuit simulation.
 *
 * This file contains the definition of the Inductor class, which models an
 * inductor element in an electrical circuit. It inherits from CircuitElement
 * and implements the required interface for stamping and parsing.
 */

/**
 * @class Inductor
 * @brief Represents an inductor element in the circuit.
 *
 * The Inductor class provides methods for stamping its contribution into the
 * MNA matrix and for parsing an inductor from a netlist.
 */
class Inductor : public CircuitElement
{
   private:
    // --- Transient state (Trapezoidal companion model) ---
    // Inductor current at previous timestep (i_n)
    double i_prev = 0.0;
    // Voltage across inductor at previous timestep (u_n) — optional but useful
    double u_prev = 0.0;
    // Companion model parameters for current time-step (computed by
    // computeCompanion)
    double Geq =
        0.0;  // equivalent conductance = h/(2*L) (or as used in your stamping)
    double Ieq = 0.0;  // equivalent current source

   public:
    /**
     * @brief Constructs an Inductor element.
     * @param name Name of the inductor element.
     * @param nodeA Starting node name.
     * @param nodeB Ending node name.
     * @param value Inductance value.
     */
    Inductor(const std::string& name, const std::string& nodeA,
             const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }
    /**
     * @brief Stamps the inductor's contribution into the MNA matrix and RHS
     * vector.
     * @param mna Modified Nodal Analysis matrix.
     * @param rhs Right-hand side vector.
     * @param indexMap Map from node/element names to matrix indices.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parses an inductor element from netlist tokens.
     * @param parser Reference to the parser.
     * @param tokens Vector of tokens from the netlist line.
     * @param lineNumber Line number in the netlist file.
     * @return Shared pointer to the created Inductor element.
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
