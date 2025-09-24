#include "VoltageSource.hpp"

#include <memory>

std::shared_ptr<CircuitElement> VoltageSource::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Validate token count
    if (!parser.validateTokens(tokens, 4, lineNumber)) {
        std::cerr << "Error: Invalid voltage source definition at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Validate nodes
    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Voltage source nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    // Parse value
    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for voltage source value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    std::shared_ptr<VoltageSource> element =
        std::make_shared<VoltageSource>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::V;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

void VoltageSource::stamp(std::vector<std::vector<double>>& mna,
                          std::vector<double>& rhs,
                          std::map<std::string, int>& indexMap)
{
    // Independent Voltage Source (always Group 2)
    if (nodeA.compare("0") == 0) {
        int vminus = indexMap[nodeB];
        int i = indexMap[name];
        mna[vminus][i] += -1.0;
        mna[i][vminus] += -1.0;
        rhs[i] += value;
    } else if (nodeB.compare("0") == 0) {
        int vplus = indexMap[nodeA];
        int i = indexMap[name];
        mna[vplus][i] += 1.0;
        mna[i][vplus] += 1.0;
        rhs[i] += value;
    } else {
        int vplus = indexMap[nodeA];
        int vminus = indexMap[nodeB];
        int i = indexMap[name];
        mna[vplus][i] += 1.0;
        mna[vminus][i] += -1.0;
        mna[i][vplus] += 1.0;
        mna[i][vminus] += -1.0;
        rhs[i] += value;
    }
}
