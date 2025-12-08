/*
 * Copyright (c) 2022, Shiv Nadar University, Delhi NCR, India. All Rights
 * Reserved. Permission to use, copy, modify and distribute this software for
 * educational, research, and not-for-profit purposes, without fee and without a
 * signed license agreement, is hereby granted, provided that this paragraph and
 * the following two paragraphs appear in all copies, modifications, and
 * distributions.
 *
 * IN NO EVENT SHALL SHIV NADAR UNIVERSITY BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST
 * PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE.
 *
 * SHIV NADAR UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS PROVIDED "AS IS". SHIV
 * NADAR UNIVERSITY HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 */
/**
 * @file NonlinearCapacitor.cpp
 * @brief Implementation of NonlinearCapacitor (nonlinear charge-based
 * capacitor).
 *
 * Implements companion computation (per-step and per-iterate), transient
 * stamping, state update, diagnostics, and checkpoint/restore helpers.
 * Comments are concise; API-level documentation is in the header.
 */

#include "NonlinearCapacitor.hpp"

#include "../lib/external/Eigen/Dense"
#include "CircuitElement.hpp"
#include "Node.hpp"

void NonlinearCapacitor::computeCompanion(double h)
{
    // Compute trapezoidal companion linearized at previous state (fallback).
    const double dqdu = model_->dqdu(u_prev_);
    Geq_ = (2.0 / h) * dqdu;
    // Ieq = Geq * u_prev - i_prev (TR Norton convention)
    Ieq_ = Geq_ * u_prev_ - i_prev_;
}

void NonlinearCapacitor::computeCompanionIter(
    double h, const Eigen::Ref<const Eigen::VectorXd> &xk,
    const std::map<std::string, int> &indexMap)
{
    // Linearize q(u) at the current Newton iterate xk. Read node voltages
    // from xk; treat ground ("0") as voltage 0.
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
    const double maxDeriv = 1e12;
    if (!std::isfinite(qk) || !std::isfinite(dqdu_k)) {
        // invalid model evaluation; bail out so caller can backtrack
        return;
    }
    if (std::abs(dqdu_k) < minDeriv)
        dqdu_k = (dqdu_k >= 0.0) ? minDeriv : -minDeriv;
    if (std::abs(dqdu_k) > maxDeriv)
        dqdu_k = (dqdu_k > 0.0) ? maxDeriv : -maxDeriv;

    // compute candidates in temporaries and validate before assigning
    double Geq_temp = (2.0 / h) * dqdu_k;
    double Ieq_temp = Geq_temp * u_k - (2.0 / h) * (qk - q_prev_) - i_prev_;

    // Cap magnitudes to avoid ill-conditioned entries
    const double maxGeq = 1e12;
    if (!std::isfinite(Geq_temp) || std::abs(Geq_temp) > maxGeq) {
        // invalid or too large -> keep previous Geq/Ieq (safer) and return
        return;
    }

    if (!std::isfinite(Ieq_temp) || std::abs(Ieq_temp) > 1e300) {
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

std::vector<double> NonlinearCapacitor::snapshotState() const
{
    // store minimal internal scalars required to restore transient state
    return {u_prev_, i_prev_, q_prev_};
}

void NonlinearCapacitor::restoreState(const std::vector<double> &data)
{
    if (data.size() >= 3) {
        u_prev_ = data[0];
        i_prev_ = data[1];
        q_prev_ = data[2];
    }
}
