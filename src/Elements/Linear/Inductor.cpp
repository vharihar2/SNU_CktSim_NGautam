#include "Inductor.hpp"

#include "Parser.hpp"

std::shared_ptr<CircuitElement> Inductor::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Inductors are always group 2
    if (!parser.validateTokens(tokens, 4, lineNumber)) {
        std::cerr << "Error: Invalid inductor definition at line " << lineNumber
                  << std::endl;
        return nullptr;
    }

    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Inductor nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for inductor value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element =
        std::make_shared<Inductor>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::L;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

void Inductor::stamp(std::vector<std::vector<double>>& mna,
                     std::vector<double>& rhs,
                     std::map<std::string, int>& indexMap)
{
    if (nodeA == "0") {
        int vminus = indexMap[nodeB];
        int i = indexMap[name];
        mna[vminus][i] += 1.0;
        mna[i][vminus] += 1.0;
    } else if (nodeB == "0") {
        int vplus = indexMap[nodeA];
        int i = indexMap[name];
        mna[vplus][i] += 1.0;
        mna[i][vplus] += 1.0;
    } else {
        int vplus = indexMap[nodeA];
        int vminus = indexMap[nodeB];
        int i = indexMap[name];
        mna[vplus][i] += 1.0;
        mna[vminus][i] += -1.0;
        mna[i][vplus] += 1.0;
        mna[i][vminus] += -1.0;
    }
}
