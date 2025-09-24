#include "Capacitor.hpp"

#include <memory>

#include "CircuitElement.hpp"

void Capacitor::stamp(std::vector<std::vector<double>>& mna,
                      std::vector<double>& rhs,
                      std::map<std::string, int>& indexMap)
{
    // Group 1: Capacitor is open circuit in DC, so do nothing
    if (group == Group::G1) {
    }
    // Group 2: For DC, typically also open, but if you need a placeholder:
    else {
    }
}

std::shared_ptr<CircuitElement> Capacitor::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Valid token counts: 4 or 5 (for optional group)
    if (!parser.validateTokens(tokens, 4, lineNumber) &&
        !parser.validateTokens(tokens, 5, lineNumber)) {
        std::cerr << "Error: Invalid capacitor definition at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Capacitor nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for capacitor value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element =
        std::make_shared<Capacitor>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::C;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}
