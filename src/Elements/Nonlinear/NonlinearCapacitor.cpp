#include "NonlinearCapacitor.hpp"

#include "../lib/external/Eigen/Dense"
#include "CircuitElement.hpp"
#include "Node.hpp"

void NonlinearCapacitor::computeCompanion(double h)
{
    // Fallback: evaluate at previous state (useful for non-iterative calls)
    // Use the stored u_prev_ and q_prev_ to compute linearized companion.
    const double dqdu = model_->dqdu(u_prev_);
    Geq_ = (2.0 / h) * dqdu;
    // Ieq consistent with TR Norton: Ieq = Geq * u_prev - i_prev
    Ieq_ = Geq_ * u_prev_ - i_prev_;
}

void NonlinearCapacitor::computeCompanionIter(
    double h, const Eigen::Ref<const Eigen::VectorXd> &xk,
    const std::map<std::string, int> &indexMap)
{
    // Determine node voltages for this element from the current Newton iterate
    // xk. Handle ground ('0') which is not present in indexMap.
    auto findIndex = [&](const std::string &n) -> int {
        if (n == "0") return -1;
        auto it = indexMap.find(n);
        return (it == indexMap.end()) ? -1 : it->second;
    };

    const int idxPlus = findIndex(nodeA);
    const int idxMinus = findIndex(nodeB);

    const double vPlus = (idxPlus >= 0) ? xk[idxPlus] : 0.0;
    const double vMinus = (idxMinus >= 0) ? xk[idxMinus] : 0.0;
    const double u_k = vPlus - vMinus;  // voltage across capacitor at iterate

    // model evaluations at u_k
    const double qk = model_->q(u_k);
    double dqdu_k = model_->dqdu(u_k);

    // Safety: clamp derivative to avoid divide-by-zero / extreme Geq values
    const double minDeriv = 1e-12;
    if (std::abs(dqdu_k) < minDeriv)
        dqdu_k = (dqdu_k >= 0.0) ? minDeriv : -minDeriv;

    // compute candidates in temporaries and validate before assigning
    double Geq_temp = (2.0 / h) * dqdu_k;
    double Ieq_temp = Geq_temp * u_k - (2.0 / h) * (qk - q_prev_) - i_prev_;

    // Cap magnitudes to avoid ill-conditioned entries
    const double maxGeq = 1e12;
    if (!std::isfinite(Geq_temp) || std::abs(Geq_temp) > maxGeq) {
        // invalid or too large -> keep previous Geq/Ieq (safer) and return
        return;
    }

    if (!std::isfinite(Ieq_temp)) {
        return;
    }

    // Accept computed companion values
    Geq_ = Geq_temp;
    Ieq_ = Ieq_temp;
}

void NonlinearCapacitor::stampTransient(std::vector<std::vector<double>> &mna,
                                        std::vector<double> &rhs,
                                        std::map<std::string, int> &indexMap)
{
    // stamp Geq between nodeA/nodeB and place Ieq into rhs with repository
    // sign convention. Handle ground (node '0') which is not present in
    // indexMap.
    auto itP = indexMap.find(nodeA);
    auto itN = indexMap.find(nodeB);
    int p = (itP == indexMap.end()) ? -1 : itP->second;
    int n = (itN == indexMap.end()) ? -1 : itN->second;

    // stamp entries depending on presence of nodes
    if (p >= 0 && n >= 0) {
        mna[p][p] += Geq_;
        mna[n][n] += Geq_;
        mna[p][n] -= Geq_;
        mna[n][p] -= Geq_;

        rhs[p] -= Ieq_;
        rhs[n] += Ieq_;
    } else if (p >= 0 && n < 0) {
        // minus node is ground
        mna[p][p] += Geq_;
        rhs[p] -= Ieq_;
        // ground node implicit: no matrix entry
    } else if (p < 0 && n >= 0) {
        // plus node is ground
        mna[n][n] += Geq_;
        rhs[n] += Ieq_;
    } else {
        // both nodes are ground? degenerate, nothing to stamp
    }
}

void NonlinearCapacitor::updateStateFromSolution(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const std::map<std::string, int> &indexMap)
{
    // Handle ground nodes which may not be present in indexMap
    auto itP = indexMap.find(nodeA);
    auto itN = indexMap.find(nodeB);
    int p = (itP == indexMap.end()) ? -1 : itP->second;
    int n = (itN == indexMap.end()) ? -1 : itN->second;
    const double vPlus = (p >= 0) ? x[p] : 0.0;
    const double vMinus = (n >= 0) ? x[n] : 0.0;

    const double u_new = vPlus - vMinus;

    // update stored states for next timestep
    u_prev_ = u_new;
    q_prev_ = model_->q(u_new);

    // For DC initialization (when companions haven't been computed yet),
    // Geq_ will be zero. In that case the physical DC capacitor current is
    // zero (open circuit). Otherwise compute from the last companion relation
    // used during a transient solve.
    if (Geq_ == 0.0) {
        i_prev_ = 0.0;
    } else {
        i_prev_ = Geq_ * u_new - Ieq_;
    }
}

void NonlinearCapacitor::dumpDiagnostics(
    std::ostream &os, const Eigen::Ref<const Eigen::VectorXd> &xk,
    const std::map<std::string, int> &indexMap) const
{
    auto idx = [&](const std::string &node) -> int {
        if (node == "0") return -1;
        auto it = indexMap.find(node);
        return (it == indexMap.end()) ? -1 : it->second;
    };

    int p = idx(nodeA);
    int n = idx(nodeB);
    double vplus = (p == -1) ? 0.0 : xk[p];
    double vminus = (n == -1) ? 0.0 : xk[n];
    double u_k = vplus - vminus;
    double q_k = model_->q(u_k);
    double dqdu_k = model_->dqdu(u_k);

    os << "CAP " << name << " (" << nodeA << "," << nodeB
       << ") t-voltage=" << u_k << " q=" << q_k << " dqdu=" << dqdu_k
       << " Geq=" << Geq_ << " Ieq=" << Ieq_ << " u_prev=" << u_prev_
       << " q_prev=" << q_prev_ << " i_prev=" << i_prev_ << std::endl;
}
