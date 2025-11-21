#include "Capacitor.hpp"

#include <memory>

#include "CircuitElement.hpp"
#include "Parser.hpp"

void Capacitor::stamp(std::vector<std::vector<double>>& mna,
                      std::vector<double>& rhs,
                      std::map<std::string, int>& indexMap)
{
    // Group 1: Capacitor is open circuit in DC, so do nothing
    if (group == Group::G1) {
    }
    // Group 2: For DC, typically also open, but if you need a placeholder:
    else {
    }
}

std::shared_ptr<CircuitElement> Capacitor::parse(
    Parser& parser, const std::vector<std::string>& tokens, int lineNumber)
{
    // Valid token counts: 4 or 5 (for optional group)
    if (!parser.validateTokens(tokens, 4, lineNumber) &&
        !parser.validateTokens(tokens, 5, lineNumber)) {
        std::cerr << "Error: Invalid capacitor definition at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    if (!parser.validateNodes(tokens[1], tokens[2], lineNumber)) {
        std::cerr << "Error: Capacitor nodes cannot be the same at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    bool validValue = false;
    double value = parser.parseValue(tokens[3], lineNumber, validValue);
    if (!validValue || value == 0) {
        std::cerr << "Error: Illegal argument for capacitor value at line "
                  << lineNumber << std::endl;
        return nullptr;
    }

    auto element =
        std::make_shared<Capacitor>(tokens[0], tokens[1], tokens[2], value);
    element->type = ElementType::C;
    element->group = Group::G2;
    element->controlling_variable = ControlVariable::none;
    element->controlling_element = nullptr;
    element->processed = false;
    return element;
}

// Copy-paste-ready implementation:

// Compute TR companion parameters for this capacitor for timestep h.
// After this call:
//   Geq = 2*C / h
//   Ieq = -i_prev - Geq * v_prev
// where v_prev and i_prev are the stored state at time n.
void Capacitor::computeCompanion(double h)
{
    // Guard against invalid timestep
    if (h <= 0.0) {
        // Keep previous Geq/Ieq (or set to 0), but avoid division by zero.
        Geq = 0.0;
        Ieq = 0.0;
        return;
    }

    // value is the capacitance C (as used across the codebase)
    Geq = 2.0 * value / h;  // Geq = 2*C / h
    // Correct trapezoidal Norton companion current sign.
    // Ieq should be Geq*v_n - i_n so that:
    //   i_{n+1} = Geq*v_{n+1} - Ieq = i_n + Geq*(v_{n+1}-v_n)
    Ieq = Geq * v_prev - i_prev;  // Ieq = Geq*v_n - i_n
}

void Capacitor::stampTransient(std::vector<std::vector<double>>& mna,
                               std::vector<double>& rhs,
                               std::map<std::string, int>& indexMap)
{
    // Companion (Norton) model: Geq between nodeA/nodeB, Ieq current source
    // Precondition: computeCompanion(h) has been called so Geq and Ieq are set.

    // Helper to get index; returns -1 for ground or missing
    auto idx = [&](const std::string& node) -> int {
        if (node == "0") return -1;
        auto it = indexMap.find(node);
        return (it == indexMap.end()) ? -1 : it->second;
    };

    int vplus = idx(nodeA);   // nodeA index, -1 if ground
    int vminus = idx(nodeB);  // nodeB index, -1 if ground

    // Stamp conductance (Geq) into MNA (handles ground cases)
    if (vplus == -1 && vminus == -1) {
        // both nodes are ground -> nothing to stamp
    } else if (vplus == -1) {
        // nodeA is ground, stamp at vminus only
        mna[vminus][vminus] += Geq;
    } else if (vminus == -1) {
        // nodeB is ground, stamp at vplus only
        mna[vplus][vplus] += Geq;
    } else {
        // neither node is ground: full stamp like a resistor
        mna[vplus][vplus] += Geq;
        mna[vplus][vminus] -= Geq;
        mna[vminus][vplus] -= Geq;
        mna[vminus][vminus] += Geq;
    }

    // Stamp the companion current Ieq into RHS.
    // Convention: if Ieq is the current from nodeA -> nodeB, then
    // we add -Ieq at nodeA (vplus) and +Ieq at nodeB (vminus).
    // (This matches earlier derivation: rhs[vplus] -= Ieq; rhs[vminus] += Ieq.)
    if (vplus != -1) {
        // Ensure rhs has correct size (assumes solver allocated it)
        rhs[vplus] -= Ieq;
    }
    if (vminus != -1) {
        rhs[vminus] += Ieq;
    }
}

void Capacitor::updateStateFromSolution(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    const std::map<std::string, int>& indexMap)
{
    // Read node voltages (treat missing or ground node as 0.0)
    double vplus = 0.0;
    double vminus = 0.0;

    if (nodeA != "0") {
        auto it = indexMap.find(nodeA);
        if (it != indexMap.end()) vplus = x[it->second];
    }

    if (nodeB != "0") {
        auto it = indexMap.find(nodeB);
        if (it != indexMap.end()) vminus = x[it->second];
    }

    // Voltage across capacitor at n+1
    double u_n1 = vplus - vminus;
    v_prev = u_n1;

    // Current at n+1 using TR companion relation:
    // i_{n+1} = Geq * v_{n+1} - Ieq
    i_prev = Geq * u_n1 - Ieq;
}
