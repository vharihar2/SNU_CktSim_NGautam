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
    // xk
    const int idxPlus = indexMap.at(nodeA);
    const int idxMinus = indexMap.at(nodeB);

    const double vPlus = xk[idxPlus];
    const double vMinus = xk[idxMinus];
    const double u_k = vPlus - vMinus;  // voltage across capacitor at iterate

    // model evaluations at u_k
    const double qk = model_->q(u_k);
    const double dqdu_k = model_->dqdu(u_k);

    // TR Norton-equivalent linearization
    Geq_ = (2.0 / h) * dqdu_k;

    // Ieq derived from TR companion for charge-based device:
    // discrete TR: q_{n+1} = q_n + (h/2) * (i_n + i_{n+1})
    // rearranged to Norton form yields the expression below. Equivalent
    // form used in repository: Ieq = Geq * u_k - (2/h)*(qk - q_prev_) - i_prev_
    Ieq_ = Geq_ * u_k - (2.0 / h) * (qk - q_prev_) - i_prev_;
}

void NonlinearCapacitor::stampTransient(std::vector<std::vector<double>> &mna,
                                        std::vector<double> &rhs,
                                        std::map<std::string, int> &indexMap)
{
    // stamp Geq between nodeA/nodeB and place Ieq into rhs with repository sign
    // conv
    const int p = indexMap.at(nodeA);
    const int n = indexMap.at(nodeB);

    // ensure matrix is large enough
    mna[p][p] += Geq_;
    mna[n][n] += Geq_;
    mna[p][n] -= Geq_;
    mna[n][p] -= Geq_;

    // rhs: subtract Ieq at p, add Ieq at n (convention: positive current from
    // p->n)
    rhs[p] -= Ieq_;
    rhs[n] += Ieq_;
}

void NonlinearCapacitor::updateStateFromSolution(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const std::map<std::string, int> &indexMap)
{
    const int p = indexMap.at(nodeA);
    const int n = indexMap.at(nodeB);
    const double vPlus = x[p];
    const double vMinus = x[n];

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
