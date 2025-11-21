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

// Precompute TR companion parameters for inductor
// Geq = h / (2*L)
// Ieq = i_prev + Geq * u_prev
void Inductor::computeCompanion(double h)
{
    // Guard against invalid timestep
    if (h <= 0.0) {
        Geq = 0.0;
        Ieq = 0.0;
        return;
    }

    // 'value' stores the inductance L
    Geq = h / (2.0 * value);      // Geq = h / (2L)
    Ieq = i_prev + Geq * u_prev;  // Ieq = i_n + Geq * u_n
}

// Stamp the TR companion for the inductor.
// This follows the repo's Group-2 branch-current stamp pattern
// and adds the companion series-equation:
//   v_{n+1} - (1/Geq) * i_{n+1} = - Ieq / Geq
// which yields branch-diagonal term mna[i][i] += -1/Geq and rhs[i] += -Ieq/Geq.
//
// Preconditions: computeCompanion(h) has been called to set Geq and Ieq.
void Inductor::stampTransient(std::vector<std::vector<double>>& mna,
                              std::vector<double>& rhs,
                              std::map<std::string, int>& indexMap)
{
    // If Geq is zero (e.g., h==0), skip companion stamping to avoid
    // div-by-zero.
    if (Geq == 0.0) return;

    // series-equivalent 'resistance' for branch equation
    double Rseries = 1.0 / Geq;  // R = 1/Geq

    // Locate indices
    int i = -1;
    auto it_i = indexMap.find(name);
    if (it_i != indexMap.end()) i = it_i->second;

    int vplus = -1;
    int vminus = -1;
    if (nodeA != "0") {
        auto it = indexMap.find(nodeA);
        if (it != indexMap.end()) vplus = it->second;
    }
    if (nodeB != "0") {
        auto it = indexMap.find(nodeB);
        if (it != indexMap.end()) vminus = it->second;
    }

    // Coupling entries: same pattern as existing Inductor::stamp (node <->
    // branch current)
    if (i != -1) {
        if (vplus != -1) {
            mna[vplus][i] += 1.0;
            mna[i][vplus] += 1.0;
        }
        if (vminus != -1) {
            // note: nodeB contribution has negative sign in original stamp
            mna[vminus][i] += -1.0;
            mna[i][vminus] += -1.0;
        }

        // Add branch diagonal and RHS based on companion algebra:
        // mna[i][i] += -Rseries  (following the repo's convention: mna[i][i] -=
        // R for series R)
        mna[i][i] += -Rseries;

        // RHS for the branch equation: -Ieq / Geq
        double rhs_branch = -Ieq / Geq;
        rhs[i] += rhs_branch;
    } else {
        // If branch-current index is missing, fall back to stamping Geq as
        // Norton between nodes (rare if parser is consistent). Use node-based
        // Norton: stamp Geq between nodeA-nodeB
        auto idx = [&](const std::string& node) -> int {
            if (node == "0") return -1;
            auto it = indexMap.find(node);
            return (it == indexMap.end()) ? -1 : it->second;
        };
        int na = idx(nodeA);
        int nb = idx(nodeB);
        if (na == -1 && nb == -1) {
            // nothing to stamp
            return;
        } else if (na == -1) {
            mna[nb][nb] += Geq;
            rhs[nb] += Ieq;  // sign consistent with earlier convention for
                             // Norton stamping
        } else if (nb == -1) {
            mna[na][na] += Geq;
            rhs[na] -= Ieq;
        } else {
            mna[na][na] += Geq;
            mna[na][nb] -= Geq;
            mna[nb][na] -= Geq;
            mna[nb][nb] += Geq;
            rhs[na] -= Ieq;
            rhs[nb] += Ieq;
        }
    }
}

// Update inductor state from the solution vector x.
// For Group-2 (branch-current unknown present) we read i_{n+1} directly from
// the solution. We also compute u_{n+1} = vplus - vminus and store it into
// u_prev.
void Inductor::updateStateFromSolution(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    const std::map<std::string, int>& indexMap)
{
    // Read node voltages
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

    double u_n1 = vplus - vminus;
    u_prev = u_n1;

    // Prefer reading branch current variable if present
    auto it_i = indexMap.find(name);
    if (it_i != indexMap.end()) {
        int i_index = it_i->second;
        i_prev = x[i_index];  // i_{n+1}
    } else {
        // Fallback: compute i_{n+1} from companion relation if branch unknown
        // not present: i_{n+1} = Geq * u_{n+1} + Ieq
        i_prev = Geq * u_n1 + Ieq;
    }
}
