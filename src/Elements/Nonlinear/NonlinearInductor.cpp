#include "NonlinearInductor.hpp"

#include "../lib/external/Eigen/Dense"
#include "CircuitElement.hpp"
#include "Node.hpp"

void NonlinearInductor::computeCompanion(double h)
{
    if (h <= 0.0) {
        Geq_ = 0.0;
        Ieq_ = 0.0;
        return;
    }

    // linear fallback using stored phi_prev_ and i_prev_ (model derivative
    // evaluated at previous current)
    const double dphidi = model_->dphidi(i_prev_);
    Geq_ = h / (2.0 * dphidi);  // Geq = h / (2 * dphidi)
    Ieq_ = i_prev_ + Geq_ * u_prev_;
}

void NonlinearInductor::computeCompanionIter(
    double h, const Eigen::Ref<const Eigen::VectorXd> &xk,
    const std::map<std::string, int> &indexMap)
{
    if (h <= 0.0) {
        Geq_ = 0.0;
        Ieq_ = 0.0;
        return;
    }

    // get branch current guess i_k if available
    double i_k = i_prev_;
    auto it_i = indexMap.find(name);
    if (it_i != indexMap.end()) {
        i_k = xk[it_i->second];
    }

    // node voltages (used for fallback or u_prev reference)
    double vplus = 0.0;
    double vminus = 0.0;
    if (nodeA != "0") {
        auto it = indexMap.find(nodeA);
        if (it != indexMap.end()) vplus = xk[it->second];
    }
    if (nodeB != "0") {
        auto it = indexMap.find(nodeB);
        if (it != indexMap.end()) vminus = xk[it->second];
    }
    double u_k = vplus - vminus;

    // evaluate model at i_k
    const double phi_k = model_->phi(i_k);
    const double dphidi_k = model_->dphidi(i_k);

    // Geq and Rseries follow dual of capacitor formulas
    Geq_ = h / (2.0 * dphidi_k);

    // Derived Ieq for flux-based TR linearization (see derivation in code
    // comments):
    // Ieq = i_k - (phi_k - phi_prev_) / dphidi_k + Geq_ * u_prev_
    Ieq_ = i_k - (phi_k - phi_prev_) / dphidi_k + Geq_ * u_prev_;
}

void NonlinearInductor::stampTransient(std::vector<std::vector<double>> &mna,
                                       std::vector<double> &rhs,
                                       std::map<std::string, int> &indexMap)
{
    if (Geq_ == 0.0) return;

    double Rseries = 1.0 / Geq_;

    // indices
    int i_index = -1;
    auto it_i = indexMap.find(name);
    if (it_i != indexMap.end()) i_index = it_i->second;

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

    if (i_index != -1) {
        if (vplus != -1) {
            mna[vplus][i_index] += 1.0;
            mna[i_index][vplus] += 1.0;
        }
        if (vminus != -1) {
            mna[vminus][i_index] += -1.0;
            mna[i_index][vminus] += -1.0;
        }

        mna[i_index][i_index] += -Rseries;

        double rhs_branch = -Ieq_ / Geq_;
        rhs[i_index] += rhs_branch;
    } else {
        // fallback to node-based Norton-style stamping if branch unknown is
        // missing
        auto idx = [&](const std::string &node) -> int {
            if (node == "0") return -1;
            auto it = indexMap.find(node);
            return (it == indexMap.end()) ? -1 : it->second;
        };
        int na = idx(nodeA);
        int nb = idx(nodeB);
        if (na == -1 && nb == -1) return;
        if (na == -1) {
            mna[nb][nb] += Geq_;
            rhs[nb] += Ieq_;
        } else if (nb == -1) {
            mna[na][na] += Geq_;
            rhs[na] -= Ieq_;
        } else {
            mna[na][na] += Geq_;
            mna[na][nb] -= Geq_;
            mna[nb][na] -= Geq_;
            mna[nb][nb] += Geq_;
            rhs[na] -= Ieq_;
            rhs[nb] += Ieq_;
        }
    }
}

void NonlinearInductor::updateStateFromSolution(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const std::map<std::string, int> &indexMap)
{
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
    u_prev_ = u_n1;

    auto it_i = indexMap.find(name);
    if (it_i != indexMap.end()) {
        int idx = it_i->second;
        i_prev_ = x[idx];
    } else {
        // fallback: compute i_{n+1} from companion relation
        i_prev_ = Geq_ * u_n1 + Ieq_;
    }

    phi_prev_ = model_->phi(i_prev_);
}

// DC stamp for inductor: create branch-current unknown and node-branch
// coupling entries following the linear Inductor::stamp pattern so the DC
// operating point provides a valid i_prev and u_prev for the nonlinear model.
void NonlinearInductor::stamp(std::vector<std::vector<double>> &mna,
                              std::vector<double> & /*rhs*/,
                              std::map<std::string, int> &indexMap)
{
    // Locate branch index (name)
    int i_index = -1;
    auto it_i = indexMap.find(name);
    if (it_i != indexMap.end()) i_index = it_i->second;

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

    if (i_index != -1) {
        if (vplus != -1) {
            mna[vplus][i_index] += 1.0;
            mna[i_index][vplus] += 1.0;
        }
        if (vminus != -1) {
            mna[vminus][i_index] += -1.0;
            mna[i_index][vminus] += -1.0;
        }
    } else {
        // no branch unknown; fallback: nothing to stamp for DC here
    }
}
