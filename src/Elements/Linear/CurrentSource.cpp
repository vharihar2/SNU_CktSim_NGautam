#include "CurrentSource.hpp"

#include <string>
#include <vector>

void CurrentSource::stamp(std::vector<std::vector<double>>& mna,
                          std::vector<double>& rhs,
                          std::map<std::string, int>& indexMap)
{
    if (group == Group::G1) {
        if (nodeA.compare("0") == 0) {
            int vminus = indexMap[nodeB];
            rhs[vminus] += value;
        } else if (nodeB.compare("0") == 0) {
            int vplus = indexMap[nodeA];
            rhs[vplus] -= value;
        } else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            rhs[vplus] -= value;
            rhs[vminus] += value;
        }
    } else {
        if (nodeA.compare("0") == 0) {
            int vminus = indexMap[nodeB];
            int i = indexMap[name];
            mna[vminus][i] = -1.0;
            mna[i][i] = +1.0;
            rhs[i] = value;
        } else if (nodeB.compare("0") == 0) {
            int vplus = indexMap[nodeA];
            int i = indexMap[name];
            mna[vplus][i] = +1.0;
            mna[i][i] = +1.0;
            rhs[i] = value;
        } else {
            int vplus = indexMap[nodeA];
            int vminus = indexMap[nodeB];
            int i = indexMap[name];
            mna[vplus][i] = +1.0;
            mna[vminus][i] = -1.0;
            mna[i][i] = +1.0;
            rhs[i] = value;
        }
    }
}
