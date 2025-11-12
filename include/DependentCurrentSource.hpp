#include "CircuitElement.hpp"

class DependentCurrentSource : public CircuitElement
{
   public:
    DependentCurrentSource(const std::string& name, const std::string& nodeA,
                           const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }
    /**
     * @brief Stamps the dependent current source into the MNA matrix and RHS
     * vector.
     * @param mna Modified Nodal Analysis matrix to be updated.
     * @param rhs Right-hand side vector to be updated.
     * @param indexMap Mapping from node names to matrix indices.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parses a DependentCurrentSource from a netlist line.
     * @param parser Reference to the Parser object.
     * @param tokens Tokenized netlist line.
     * @param lineNumber Line number in the netlist file.
     * @return Shared pointer to the created DependentCurrentSource object.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
};
