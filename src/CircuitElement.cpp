#include "CircuitElement.hpp"

#include <set>
#include <string>

int CircuitElement::resolveDependencies(
    std::vector<std::shared_ptr<CircuitElement>> &circuitElements,
    std::map<std::string, std::shared_ptr<CircuitElement>> &elementMap,
    std::set<std::string> &nodes_group2)
{
    int erroCount = 0;
    for (auto &elem : circuitElements) {
        if (elem->controlling_variable != ControlVariable::none &&
            elem->controlling_element) {
            auto it = elementMap.find(elem->controlling_element->name);
            if (it != elementMap.end()) {
                elem->controlling_element = it->second;
                if (elem->controlling_variable == ControlVariable::i &&
                    elem->controlling_element->group != Group::G2) {
                    std::cerr << "Warning: Referenced element "
                              << elem->controlling_element->name
                              << " must be in group 2 as its current variable "
                                 "is required by "
                              << elem->name << std::endl;
                    elem->controlling_element->group = Group::G2;
                    nodes_group2.insert(elem->controlling_element->name);
                }
            } else {
                std::cerr << "Error: Referenced element "
                          << elem->controlling_element->name
                          << " (referenced by " << elem->name
                          << ") is not present in the netlist" << std::endl;
                erroCount++;
            }
        }
    }
    return erroCount;
}
