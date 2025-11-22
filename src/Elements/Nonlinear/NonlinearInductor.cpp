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
    auto idx = [&](const std::string &node) -> int {
        if (node == "0") return -1;
        auto it = indexMap.find(node);
        return (it == indexMap.end()) ? -1 : it->second;
    };

    double vplus = 0.0;
    double vminus = 0.0;
    int pidx = idx(nodeA);
    int nidx = idx(nodeB);
    if (pidx != -1) vplus = xk[pidx];
    if (nidx != -1) vminus = xk[nidx];
    double u_k = vplus - vminus;

    // evaluate model at i_k
    const double phi_k = model_->phi(i_k);
    const double dphidi_k = model_->dphidi(i_k);

    // Geq and Rseries follow dual of capacitor formulas
    // Safety: clamp derivative to avoid divide-by-zero / extreme values
    double dphidi_safe = dphidi_k;
    const double minDeriv = 1e-12;
    if (std::abs(dphidi_safe) < minDeriv)
        dphidi_safe = (dphidi_safe >= 0.0) ? minDeriv : -minDeriv;

    // compute temporaries and validate
    double Geq_temp = h / (2.0 * dphidi_safe);
    double Ieq_temp =
        i_k - (phi_k - phi_prev_) / dphidi_safe + Geq_temp * u_prev_;

    const double maxGeq = 1e12;
    if (!std::isfinite(Geq_temp) || std::abs(Geq_temp) > maxGeq) {
        return;  // keep previous values if new ones are invalid
    }
    if (!std::isfinite(Ieq_temp)) {
        return;
    }

    Geq_ = Geq_temp;
    Ieq_ = Ieq_temp;
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

    auto idx = [&](const std::string &node) -> int {
        if (node == "0") return -1;
        auto it = indexMap.find(node);
        return (it == indexMap.end()) ? -1 : it->second;
    };

    int vplus = idx(nodeA);
    int vminus = idx(nodeB);

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
    auto idx = [&](const std::string &node) -> int {
        if (node == "0") return -1;
        auto it = indexMap.find(node);
        return (it == indexMap.end()) ? -1 : it->second;
    };

    double vplus = 0.0;
    double vminus = 0.0;
    int pidx = idx(nodeA);
    int nidx = idx(nodeB);
    if (pidx != -1) vplus = x[pidx];
    if (nidx != -1) vminus = x[nidx];

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

void NonlinearInductor::dumpDiagnostics(
    std::ostream &os, const Eigen::Ref<const Eigen::VectorXd> &xk,
    const std::map<std::string, int> &indexMap) const
{
    auto idx = [&](const std::string &node) -> int {
        if (node == "0") return -1;
        auto it = indexMap.find(node);
        return (it == indexMap.end()) ? -1 : it->second;
    };

    // branch current variable stored under element name if present
    double i_k = i_prev_;
    auto it_i = indexMap.find(name);
    if (it_i != indexMap.end()) i_k = xk[it_i->second];

    int pidx = idx(nodeA);
    int nidx = idx(nodeB);
    double vplus = (pidx == -1) ? 0.0 : xk[pidx];
    double vminus = (nidx == -1) ? 0.0 : xk[nidx];
    double u_k = vplus - vminus;

    double phi_k = model_->phi(i_k);
    double dphidi_k = model_->dphidi(i_k);

    os << "IND " << name << " (" << nodeA << "," << nodeB << ") i=" << i_k
       << " phi=" << phi_k << " dphidi=" << dphidi_k << " Geq=" << Geq_
       << " Ieq=" << Ieq_ << " u_prev=" << u_prev_ << " phi_prev=" << phi_prev_
       << std::endl;
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
