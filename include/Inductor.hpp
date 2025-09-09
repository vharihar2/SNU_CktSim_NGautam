#include "CircuitElement.hpp"
class Inductor : public CircuitElement
{
   public:
    Inductor(const std::string& name, std::string& nodeA, std::string& nodeB,
             double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;
};
