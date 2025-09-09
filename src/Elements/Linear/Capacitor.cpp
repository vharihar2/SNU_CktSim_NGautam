#include "Capacitor.hpp"

#include "CircuitElement.hpp"

void Capacitor::stamp(std::vector<std::vector<double>>& mna,
                      std::vector<double>& rhs,
                      std::map<std::string, int>& indexMap)
{
    // Group 1: Capacitor is open circuit in DC, so do nothing
    if (group == Group::G1) {
    }
    // Group 2: For DC, typically also open, but if you need a placeholder:
    else {
        int i = indexMap[name];
        mna[i][i] += 1.0;
    }
}
