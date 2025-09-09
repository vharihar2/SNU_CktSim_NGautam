#include "Resistor.hpp"

#include <map>
#include <string>
#include <vector>

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
