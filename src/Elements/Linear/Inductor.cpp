#include "Inductor.hpp"

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
