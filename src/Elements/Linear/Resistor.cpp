#include "Resistor.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "CircuitElement.hpp"
#include "Parser.hpp"

std::shared_ptr<CircuitElement> Resistor::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Validate token count
    if (!parser.validateTokens(tokens, 4, lineNumber) &&
        !parser.validateTokens(tokens, 5, lineNumber)) {
        std::cerr << "Error: Invalid resistor definition at line " << lineNumber
                  << std::endl;
        return nullptr;
    }

    // Validate nodes
    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Resistor nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Parse value
    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for resistor value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Create resistor element
    auto element =
        std::make_shared<Resistor>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::R;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

void Resistor::stamp(std::vector<std::vector<double>>& mna,
                     std::vector<double>& rhs,
                     std::map<std::string, int>& indexMap)
{
    if (group == Group::G1) {
        if (nodeA == "0") {
            int vminus = indexMap[nodeB];
            mna[vminus][vminus] += 1.0 / value;
        } else if (nodeB == "0") {
            int vplus = indexMap[nodeA];
            mna[vplus][vplus] += 1.0 / value;
        } else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            double conductance = 1.0 / value;
            mna[vplus][vplus] += conductance;
            mna[vplus][vminus] -= conductance;
            mna[vminus][vplus] -= conductance;
            mna[vminus][vminus] += conductance;
        }
    }
    // Group 2: Resistor current as variable
    else {
        if (nodeA == "0") {
            int vminus = indexMap[nodeB];
            int i = indexMap[name];
            mna[vminus][i] -= 1.0;
            mna[i][vminus] -= 1.0;
            mna[i][i] -= value;
        } else if (nodeB == "0") {
            int vplus = indexMap[nodeA];
            int i = indexMap[name];
            mna[vplus][i] += 1.0;
            mna[i][vplus] += 1.0;
            mna[i][i] -= value;
        } else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            int i = indexMap[name];
            mna[vplus][i] += 1.0;
            mna[vminus][i] -= 1.0;
            mna[i][vplus] += 1.0;
            mna[i][vminus] -= 1.0;
            mna[i][i] -= value;
        }
    }
}
