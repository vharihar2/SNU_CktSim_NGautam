#include <memory>
#include <string>
#include <vector>

#include "CircuitElement.hpp"
#include "Parser.hpp"

class Resistor : public CircuitElement
{
   public:
    Resistor(const std::string& name, std::string& nodeA, std::string& nodeB,
             double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }

    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);
};
