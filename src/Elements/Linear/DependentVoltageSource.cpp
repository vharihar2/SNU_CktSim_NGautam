#include "DependentVoltageSource.hpp"

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
