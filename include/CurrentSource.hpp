#include "CircuitElement.hpp"
class CurrentSource : public CircuitElement
{
   public:
    CurrentSource(const std::string& name, std::string& nodeA,
                  std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;
};
