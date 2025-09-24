#include "CurrentSource.hpp"

#include <memory>
#include <string>
#include <vector>

std::shared_ptr<CircuitElement> CurrentSource::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Valid token counts: 4 or 5 (for optional group)
    if (!parser.validateTokens(tokens, 4, lineNumber) &&
        !parser.validateTokens(tokens, 5, lineNumber)) {
        std::cerr << "Error: Invalid current source definition at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Current source nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for current source value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element =
        std::make_shared<CurrentSource>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::I;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

// void CurrentSource::stamp(std::vector<std::vector<double>>& mna,
//                           std::vector<double>& rhs,
//                           std::map<std::string, int>& indexMap)
// {
//     if (group == Group::G1) {
//         if (nodeA.compare("0") == 0) {
//             int vminus = indexMap[nodeB];
//             rhs[vminus] += value;
//         } else if (nodeB.compare("0") == 0) {
//             int vplus = indexMap[nodeA];
//             rhs[vplus] -= value;
//         } else {
//             int vplus = indexMap[nodeA];
//             int vminus = indexMap[nodeB];
//             rhs[vplus] -= value;
//             rhs[vminus] += value;
//         }
//     } else {
//         if (nodeA.compare("0") == 0) {
//             int vminus = indexMap[nodeB];
//             int i = indexMap[name];
//             mna[vminus][i] = -1.0;
//             mna[i][i] = +1.0;
//             rhs[i] = value;
//         } else if (nodeB.compare("0") == 0) {
//             int vplus = indexMap[nodeA];
//             int i = indexMap[name];
//             mna[vplus][i] = +1.0;
//             mna[i][i] = +1.0;
//             rhs[i] = value;
//         } else {
//             int vplus = indexMap[nodeA];
//             int vminus = indexMap[nodeB];
//             int i = indexMap[name];
//             mna[vplus][i] = +1.0;
//             mna[vminus][i] = -1.0;
//             mna[i][i] = +1.0;
//             rhs[i] = value;
//         }
//     }
// }

void CurrentSource::stamp(std::vector<std::vector<double>>& mna,
                          std::vector<double>& rhs,
                          std::map<std::string, int>& indexMap)
{
    // For DC MNA, current sources only affect the RHS vector.
    // No new unknowns, no matrix stamp.
    if (nodeA != "0") {
        int vplus = indexMap[nodeA];
        rhs[vplus] -= value;
    }
    if (nodeB != "0") {
        int vminus = indexMap[nodeB];
        rhs[vminus] += value;
    }
}
