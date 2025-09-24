#include "DependentVoltageSource.hpp"

std::shared_ptr<CircuitElement> DependentVoltageSource::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Must have at least 6 tokens: VC name nodeA nodeB value
    // controlling_variable controlling_element
    if (!parser.validateTokens(tokens, 6, lineNumber)) {
        std::cerr
            << "Error: Invalid dependent voltage source definition at line "
            << lineNumber << std::endl;
        return nullptr;
    }

    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Dependent voltage source nodes cannot be the same "
                     "at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for dependent voltage source "
                     "value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element = std::make_shared<DependentVoltageSource>(
        tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::Vc;
    element->group = Group::G2;

    // Controlling variable: must be "V" or "I"
    if (tokens[4] == "V") {
        element->controlling_variable = ControlVariable::v;
    } else if (tokens[4] == "I") {
        element->controlling_variable = ControlVariable::i;
    } else {
        std::cerr << "Error: Illegal controlling variable argument at line "
                  << lineNumber << std::endl;
        element->controlling_variable = ControlVariable::none;
    }

    // Cascading check: controlling element cannot be VC or IC
    if (tokens[5].find("IC") == 0 || tokens[5].find("VC") == 0) {
        std::cerr << "Error: Controlled source " << tokens[0]
                  << " cannot be cascaded at line " << lineNumber << std::endl;
        element->controlling_variable = ControlVariable::none;
        element->controlling_element = nullptr;
    } else {
        element->controlling_element = nullptr;
    }
    return element;
}

void DependentVoltageSource::stamp(std::vector<std::vector<double>>& mna,
                                   std::vector<double>& rhs,
                                   std::map<std::string, int>& indexMap)
{
    // Dependant Voltage Source (always Group 2)
    // Current Controlled Voltage Source (CCVS)
    if (controlling_variable == ControlVariable::i) {
        if (nodeA.compare("0") == 0) {
            int vminus = indexMap[nodeB];
            int is = indexMap[name];
            int ix = indexMap[controlling_element->getName()];
            mna[vminus][is] += -1.0;
            mna[is][vminus] += -1.0;
            mna[is][ix] += -value;
        } else if (nodeB.compare("0") == 0) {
            int vplus = indexMap[nodeA];
            int is = indexMap[name];
            int ix = indexMap[controlling_element->getName()];
            mna[vplus][is] += 1.0;
            mna[is][vplus] += 1.0;
            mna[is][ix] += -value;
        } else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            int is = indexMap[name];
            int ix = indexMap[controlling_element->getName()];
            mna[vplus][is] += 1.0;
            mna[vminus][is] += -1.0;
            mna[is][vplus] += 1.0;
            mna[is][vminus] += -1.0;
            mna[is][ix] += -value;
        }
    }
    // Voltage Controlled Voltage Source (VCVS)
    else {
        if (nodeA.compare("0") == 0) {
            int vminus = indexMap[nodeB];
            // When controlling_element's source node is connected to ground
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                int i = indexMap[name];
                mna[vminus][i] += -1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxminus] += value;
            }
            // When controlling_element's target node is connected to ground
            else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int i = indexMap[name];
                mna[vminus][i] += -1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                int i = indexMap[name];
                mna[vminus][i] += -1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -value;
                mna[i][vxminus] += value;
            }
        }
        // When target node is connected to ground
        else if (nodeB.compare("0") == 0) {
            int vplus = indexMap[nodeA];
            // When controlling_element's source node is connected to ground
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                int i = indexMap[name];
                mna[vplus][i] += 1.0;
                mna[i][vplus] += 1.0;
                mna[i][vxminus] += value;
            }
            // When controlling_element's target node is connected to ground
            else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int i = indexMap[name];
                mna[vplus][i] += 1.0;
                mna[i][vplus] += 1.0;
                mna[i][vxplus] += -value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                int i = indexMap[name];
                mna[vplus][i] += 1.0;
                mna[i][vplus] += 1.0;
                mna[i][vxplus] += -value;
                mna[i][vxminus] += value;
            }
        }
        // When both the nodes are not connected ground
        else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            // When controlling_element's source node is connected to ground
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                int i = indexMap[name];
                mna[vplus][i] += 1.0;
                mna[vminus][i] += -1.0;
                mna[i][vplus] += 1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxminus] += value;
            }
            // When controlling_element's target node is connected to ground
            else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int i = indexMap[name];
                mna[vplus][i] += 1.0;
                mna[vminus][i] += -1.0;
                mna[i][vplus] += 1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                int i = indexMap[name];
                mna[vplus][i] += 1.0;
                mna[vminus][i] += -1.0;
                mna[i][vplus] += 1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -value;
                mna[i][vxminus] += value;
            }
        }
    }
}
