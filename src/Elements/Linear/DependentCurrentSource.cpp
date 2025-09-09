#include "DependentCurrentSource.hpp"

void DependentCurrentSource::stamp(std::vector<std::vector<double>>& mna,
                                   std::vector<double>& rhs,
                                   std::map<std::string, int>& indexMap)
{
    // Dependant Current Source (always Group 1)
    // Current Controlled Current Source (CCCS)
    if (controlling_variable == ControlVariable::i) {
        if (nodeA.compare("0") == 0) {
            int vminus = indexMap[nodeB];
            int i = indexMap[controlling_element->getName()];
            mna[vminus][i] += -value;
        } else if (nodeB.compare("0") == 0) {
            int vplus = indexMap[nodeA];
            int i = indexMap[controlling_element->getName()];
            mna[vplus][i] += value;
        } else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            int i = indexMap[controlling_element->getName()];
            mna[vplus][i] += value;
            mna[vminus][i] += -value;
        }
    }
    // Voltage Controlled Current Source (VCCS)
    else {
        if (nodeA.compare("0") == 0) {
            int vminus = indexMap[nodeB];
            // When controlling_element's source node is connected to ground
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vminus][vxminus] += value;
            }
            // When controlling_element's target node is connected to ground
            else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                mna[vminus][vxplus] += -value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vminus][vxplus] += -value;
                mna[vminus][vxminus] += value;
            }
        }
        // When target node is connected to ground
        else if (nodeB.compare("0") == 0) {
            int vplus = indexMap[nodeA];
            // When controlling_element's source node is connected to ground
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][vxminus] += -value;
            }
            // When controlling_element's target node is connected to ground
            else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                mna[vplus][vxplus] += value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][vxplus] += value;
                mna[vplus][vxminus] += -value;
            }
        }
        // When both the nodes are not connected ground
        else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            // When controlling_element's source node is connected to ground
            if (controlling_element->getNodeA() == "0") {
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][vxminus] += -value;
                mna[vminus][vxminus] += value;
            }
            // When controlling_element's target node is connected to ground
            else if (controlling_element->getNodeB() == "0") {
                int vxplus = indexMap[controlling_element->getNodeA()];
                mna[vplus][vxplus] += value;
                mna[vminus][vxplus] += -value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus = indexMap[controlling_element->getNodeA()];
                int vxminus = indexMap[controlling_element->getNodeB()];
                mna[vplus][vxplus] += value;
                mna[vplus][vxminus] += -value;
                mna[vminus][vxplus] += -value;
                mna[vminus][vxminus] += value;
            }
        }
    }
}
