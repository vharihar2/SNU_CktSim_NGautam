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
 * @file main.cpp
 *
 * @brief Contains the  implemntation of the main functions
 */

#include "Solver.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

#include "Edge.hpp"

// Step checkpoint helpers for adaptive timestep
struct ElementSnapshot
{
    std::string name;
    std::vector<double> data;
};

struct StepCheckpoint
{
    double t;
    double h;
    Eigen::VectorXd x0;
    std::vector<ElementSnapshot> elSnapshots;
};

// Forward declarations for functions defined later in this file but used
// by helpers placed earlier (keeps ordering stable across edits).
static void resetProcessedFlags(
    std::map<std::string, std::shared_ptr<Node>> &nodeMap,
    std::vector<std::shared_ptr<CircuitElement>> &circuitElements);

static StepCheckpoint createCheckpoint(
    Parser &parser, const Eigen::Ref<const Eigen::VectorXd> &x0, double t,
    double h)
{
    StepCheckpoint cp;
    cp.t = t;
    cp.h = h;
    cp.x0 = x0;
    cp.elSnapshots.reserve(parser.circuitElements.size());
    for (auto &el : parser.circuitElements) {
        if (!el) continue;
        ElementSnapshot s;
        s.name = el->getName();
        s.data = el->snapshotState();
        cp.elSnapshots.push_back(std::move(s));
    }
    return cp;
}

static void restoreCheckpoint(Parser &parser, const StepCheckpoint &cp,
                              Eigen::Ref<Eigen::VectorXd> x0_out)
{
    // restore vector
    if (x0_out.size() == cp.x0.size()) x0_out = cp.x0;

    // restore element snapshots by name (match element names)
    for (const auto &s : cp.elSnapshots) {
        // find element with matching name
        for (auto &el : parser.circuitElements) {
            if (!el) continue;
            if (el->getName() == s.name) {
                el->restoreState(s.data);
                break;
            }
        }
    }
}

// Result for a single TR step attempt (non-committing)
struct SingleStepResult
{
    bool success = false;
    Eigen::VectorXd x;
    int iterations = 0;
    double residualNorm = std::numeric_limits<double>::infinity();
};

// Perform a single TR step (Newton iterations) starting from x_start at time
// t_start with step h. This function will NOT commit element states — it
// snapshots element states at entry and restores them before returning so the
// caller can decide whether to accept the result.
static SingleStepResult computeSingleStep(
    Parser &parser, std::map<std::string, std::shared_ptr<Node>> &nodeMap,
    std::map<std::string, int> &indexMap, double t_start, double h,
    const Eigen::Ref<const Eigen::VectorXd> &x_start,
    const SolverOptions &options)
{
    SingleStepResult res;

    // Create a checkpoint so we can restore element/internal state on exit
    StepCheckpoint cp = createCheckpoint(parser, x_start, t_start, h);

    int n = static_cast<int>(indexMap.size());
    if (n == 0) {
        res.success = true;
        res.x.resize(0);
        return res;
    }

    // Local solver parameters (from options where provided)
    const int maxNewtonIterations = options.maxNewtonIters;
    const double tolAbs = 1e-9;
    const double tolRel = 1e-6;
    const double tolResidual = 1e-8;
    const double newtonAlpha = options.newtonAlpha;

    std::vector<double> rhs_std(n, 0.0);
    std::vector<std::vector<double>> mna_dummy(n, std::vector<double>(n, 0.0));
    Eigen::VectorXd b(n);

    Eigen::VectorXd xk = x_start;
    if (xk.size() != n) xk = Eigen::VectorXd::Zero(n);

    for (int iter = 0; iter < maxNewtonIterations; ++iter) {
        // 1) For nonlinear elements compute companions at current iterate
        for (auto &el : parser.circuitElements) {
            if (el) el->computeCompanionIter(h, xk, indexMap);
        }

        // 2) Assemble A_k and b_k for this Newton iterate
        std::fill(rhs_std.begin(), rhs_std.end(), 0.0);
        for (auto &row : mna_dummy) std::fill(row.begin(), row.end(), 0.0);
        resetProcessedFlags(nodeMap, parser.circuitElements);
        for (auto &p : nodeMap) {
            auto nodePtr = p.second;
            if (!nodePtr) continue;
            if (nodePtr->name == "0") continue;
            if (!nodePtr->processed) {
                nodePtr->traverse(indexMap, mna_dummy, rhs_std, true);
            }
        }

        // convert mna_dummy -> Eigen A_k
        std::vector<double> flat;
        flat.reserve(n * n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) flat.push_back(mna_dummy[i][j]);
        }
        Eigen::MatrixXd A_k;
        if (n > 0) {
            A_k.resize(n, n);
            A_k = Eigen::MatrixXd::Map(flat.data(), n, n).transpose();
        } else {
            A_k.resize(0, 0);
        }

        b = Eigen::Map<Eigen::VectorXd>(rhs_std.data(), n);

        // Early sanity checks on assembled matrix/vector: reject this
        // Newton iterate early if assembly produced non-finite or
        // astronomically large values. This prevents attempting an LU
        // factorization that will produce Inf/NaN and helps the adaptive
        // retry logic reduce the timestep sooner.
        bool assemblyOk = true;
        double maxA = 0.0;
        double maxB = 0.0;
        try {
            if (A_k.size() > 0) maxA = A_k.cwiseAbs().maxCoeff();
        } catch (...) { assemblyOk = false; }
        try {
            if (b.size() > 0) maxB = b.cwiseAbs().maxCoeff();
        } catch (...) { assemblyOk = false; }

        if (!assemblyOk || !std::isfinite(maxA) || !std::isfinite(maxB) ||
            maxA > options.maxState * 1e6 || maxB > options.maxState * 1e6) {
            if (options.diagVerbose) {
                std::ofstream diag(options.diagFile, std::ios::app);
                if (diag) {
                    diag << "computeSingleStep: rejecting iterate due to bad A/b assembly" << "\n";
                    diag << "A_k.maxAbs=" << maxA << " b.maxAbs=" << maxB << "\n";
                }
            }
            // treat as failed iterate so caller will reduce step
            continue;
        }

        // 3) Solve A_k * x_new = b with LU/solve guards.
        Eigen::VectorXd x_new(n);
        if (A_k.size() > 0) {
            Eigen::FullPivLU<Eigen::MatrixXd> lu_k;
            lu_k.compute(A_k);

            if (!lu_k.isInvertible()) {
                // Try small regularizations
                const double regs[3] = {1e-12, 1e-9, 1e-6};
                bool solved = false;
                for (double rv : regs) {
                    Eigen::MatrixXd A_reg = A_k;
                    for (int d = 0; d < n; ++d) A_reg(d, d) += rv;
                    lu_k.compute(A_reg);
                    if (lu_k.isInvertible()) {
                        x_new = lu_k.solve(b);
                        bool ok = true;
                        for (int i = 0; i < n; ++i)
                            if (!std::isfinite(x_new[i])) {
                                ok = false;
                                break;
                            }
                        if (ok) {
                            solved = true;
                            break;
                        }
                    }
                }
                if (!solved) {
                    // treat as failed iterate
                    continue;
                }
            } else {
                x_new = lu_k.solve(b);
            }
        } else {
            x_new.setZero();
        }

        // 4) Under-relaxation + backtracking (choose best finite residual)
        bool xnewFinite = true;
        for (int i = 0; i < n; ++i)
            if (!std::isfinite(x_new[i])) {
                xnewFinite = false;
                break;
            }
        if (!xnewFinite) continue;
        // Early reject if linear solve produced an overly large candidate
        double maxAbsNew = 0.0;
        for (int i = 0; i < n; ++i) maxAbsNew = std::max(maxAbsNew, std::abs(x_new[i]));
        if (maxAbsNew > options.maxState * 10.0) continue;

        double prevResidualNorm = std::numeric_limits<double>::infinity();
        if (A_k.size() > 0) {
            Eigen::VectorXd r_k = A_k * xk - b;
            prevResidualNorm = r_k.norm();
        }

        const int maxBacktracks = options.maxBacktracks;
        double bestResidual = std::numeric_limits<double>::infinity();
        Eigen::VectorXd best_x_relaxed = xk;
        double tryAlpha = newtonAlpha;
        bool foundFinite = false;

        for (int bt = 0; bt <= maxBacktracks; ++bt) {
            Eigen::VectorXd x_relaxed_try = xk + tryAlpha * (x_new - xk);
            bool xrelFinite = true;
            for (int i = 0; i < n; ++i)
                if (!std::isfinite(x_relaxed_try[i])) {
                    xrelFinite = false;
                    break;
                }
            // Early growth guard: if the relaxed iterate grows wildly compared
            // to the previous iterate, treat it as non-finite and backtrack
            if (xrelFinite) {
                double maxAbs = 0.0;
                for (int i = 0; i < n; ++i) maxAbs = std::max(maxAbs, std::abs(x_relaxed_try[i]));
                const double growthFactorLimit = 1e3; // permissive but protective
                if (maxAbs > options.maxState * 10.0 || x_relaxed_try.norm() > growthFactorLimit * (1.0 + xk.norm())) {
                    xrelFinite = false;
                }
            }
            if (!xrelFinite) {
                tryAlpha *= 0.5;
                continue;
            }

            double residualNorm = std::numeric_limits<double>::infinity();
            if (A_k.size() > 0) {
                Eigen::VectorXd r = A_k * x_relaxed_try - b;
                residualNorm = r.norm();
            }
            if (!std::isfinite(residualNorm)) {
                tryAlpha *= 0.5;
                continue;
            }

            if (residualNorm < bestResidual) {
                bestResidual = residualNorm;
                best_x_relaxed = x_relaxed_try;
            }
            foundFinite = true;
            if (std::isfinite(prevResidualNorm) &&
                residualNorm <= prevResidualNorm)
                break;
            tryAlpha *= 0.5;
        }
        if (!foundFinite) continue;

        Eigen::VectorXd x_relaxed = best_x_relaxed;
        Eigen::VectorXd delta = x_relaxed - xk;
        double deltaNorm = delta.norm();

        double residualNorm = 0.0;
        if (A_k.size() > 0) {
            Eigen::VectorXd r = A_k * x_relaxed - b;
            residualNorm = r.norm();
        }

        if (std::isfinite(deltaNorm) && std::isfinite(residualNorm) &&
            (deltaNorm <= tolAbs || deltaNorm <= tolRel * (1.0 + xk.norm()) ||
             residualNorm <= tolResidual)) {
            // Additional safety: reject candidate if any state magnitude is
            // unphysically large (prevents committing divergent solutions
            // even if the LTE/controller thinks the step is "small").
            bool tooLarge = false;
            for (int i = 0; i < n; ++i) {
                if (!std::isfinite(x_relaxed[i]) ||
                    std::abs(x_relaxed[i]) > options.maxState) {
                    tooLarge = true;
                    break;
                }
            }
            if (tooLarge) {
                // treat as failed iterate so the caller will retry with
                // reduced step (or ultimately abort)
                if (options.diagVerbose) {
                    std::ofstream diag(options.diagFile, std::ios::app);
                    if (diag) {
                        diag << "computeSingleStep: rejecting candidate due to "
                             << "excessive state magnitude\n";
                    }
                }
                // continue Newton iterations (do not commit)
            } else {
                res.success = true;
                res.x = x_relaxed;
                res.iterations = iter + 1;
                res.residualNorm = residualNorm;
                // restore element states to original (non-committing)
                Eigen::VectorXd tmp = cp.x0;
                restoreCheckpoint(parser, cp, tmp);
                return res;
            }
        }

        xk = x_relaxed;
    }

    // not converged; return last iterate (do not commit states)
    res.success = false;
    res.x = xk;
    res.iterations = maxNewtonIterations;
    // try compute a residual for reporting
    try {
        Eigen::MatrixXd A_snap =
            assembleMatrixOnly(parser, nodeMap, indexMap, h);
        if (A_snap.size() > 0) {
            Eigen::VectorXd b_snap =
                Eigen::Map<Eigen::VectorXd>(rhs_std.data(), n);
            Eigen::VectorXd r = A_snap * res.x - b_snap;
            res.residualNorm = r.norm();
        }
    } catch (...) {
    }

    // restore element states
    Eigen::VectorXd tmp = cp.x0;
    restoreCheckpoint(parser, cp, tmp);
    return res;
}

// Perform two consecutive half-steps of size h/2 starting from x_start at
// time t_start. This uses the non-committing `computeSingleStep` helper to
// run each half-step while ensuring the solver's element internal states are
// not permanently modified by this routine. Returns a SingleStepResult where
// `x` is the solution after two half-steps (x_half) and `success` is true
// only if both half-steps converged.
static SingleStepResult computeTwoHalfSteps(
    Parser &parser, std::map<std::string, std::shared_ptr<Node>> &nodeMap,
    std::map<std::string, int> &indexMap, double t_start, double h,
    const Eigen::Ref<const Eigen::VectorXd> &x_start,
    const SolverOptions &options)
{
    SingleStepResult res;

    // Create a checkpoint to restore solver/element state on exit
    StepCheckpoint cp = createCheckpoint(parser, x_start, t_start, h);

    int n = static_cast<int>(indexMap.size());
    if (n == 0) {
        res.success = true;
        res.x.resize(0);
        return res;
    }

    double h2 = h * 0.5;

    // First half-step (non-committing)
    SingleStepResult s1 =
        computeSingleStep(parser, nodeMap, indexMap, t_start, h2, x_start, options);
    if (!s1.success) {
        // restore and return failure
        Eigen::VectorXd tmp = cp.x0;
        restoreCheckpoint(parser, cp, tmp);
        return s1;
    }

    // Commit first half-step into element states so second half-step sees the
    // updated internal state. We commit only temporarily; we'll restore the
    // original checkpoint before returning.
    for (auto &el : parser.circuitElements) {
        if (el) el->updateStateFromSolution(s1.x, indexMap);
    }

    // Second half-step starting at t_start + h/2 with initial guess s1.x
    SingleStepResult s2 =
        computeSingleStep(parser, nodeMap, indexMap, t_start + h2, h2, s1.x, options);

    // Restore original checkpoint (do not commit changes here)
    Eigen::VectorXd tmp = cp.x0;
    restoreCheckpoint(parser, cp, tmp);

    // Aggregate results: success only if both succeeded
    res.success = (s1.success && s2.success);
    // prefer s2.x as the final x_half when available
    res.x = s2.x.size() == n ? s2.x : s1.x;
    res.iterations = s1.iterations + s2.iterations;
    // residualNorm: choose s2 if available else s1
    res.residualNorm =
        std::isfinite(s2.residualNorm) ? s2.residualNorm : s1.residualNorm;

    return res;
}

// Estimate the local truncation error (LTE) from a full-step solution
// (`x_full`) and a two-half-step solution (`x_half`). Uses the weighted
// RMS norm defined by denom_i = atol + rtol * max(|x_half_i|, |x_start_i|).
// Returns the scalar error norm (err_norm). If vector sizes mismatch or
// denominator is zero, function handles gracefully.
static double estimateLTE(const Eigen::Ref<const Eigen::VectorXd> &x_full,
                          const Eigen::Ref<const Eigen::VectorXd> &x_half,
                          const Eigen::Ref<const Eigen::VectorXd> &x_start,
                          double atol, double rtol)
{
    // basic size checks
    if (x_full.size() != x_half.size() || x_full.size() != x_start.size()) {
        // incompatible sizes: return large error to be conservative
        return std::numeric_limits<double>::infinity();
    }

    int n = static_cast<int>(x_full.size());
    if (n == 0) return 0.0;

    double accum = 0.0;
    for (int i = 0; i < n; ++i) {
        double e = x_half[i] - x_full[i];
        double denom =
            atol + rtol * std::max(std::abs(x_half[i]), std::abs(x_start[i]));
        if (denom <= 0.0) denom = atol;  // fallback guard
        double term = e / denom;
        if (!std::isfinite(term)) {
            return std::numeric_limits<double>::infinity();
        }
        accum += term * term;
    }

    double mean = accum / static_cast<double>(n);
    double err_norm = std::sqrt(mean);
    return err_norm;
}

// Compute step-size multiplication factor from error norm using a PI-like
// controller for a method of order p. For TR, p=2, so exponent = 1/(p+1)=1/3.
// The returned factor is clamped to [fac_min, fac_max].
static double computeStepFactor(double err_norm, double safety, double fac_min,
                                double fac_max, double p)
{
    if (!std::isfinite(err_norm) || err_norm <= 0.0) {
        // If error not finite, be conservative and reduce step
        return fac_min;
    }
    double exponent = 1.0 / (p + 1.0);
    double factor = safety * std::pow(err_norm, -exponent);
    if (factor != factor) return fac_min;  // NaN guard
    factor = std::max(fac_min, std::min(fac_max, factor));
    return factor;
}

void makeIndexMap(std::map<std::string, int> &indexMap, Parser &parser)
{
    int i = 0;

    for (std::string str : parser.nodes_group2) {
        if (str.compare("0") != 0) {
            indexMap[str] = i++;
        }
    }
}

void printMNAandRHS(std::vector<std::vector<double>> &mna,
                    std::map<std::string, int> &indexMap,
                    std::vector<double> &rhs)
{
    std::cout << std::fixed;
    std::cout << std::setprecision(5);

    std::map<std::string, int>::iterator k = indexMap.begin();

    for (size_t i = 0; i < mna.size(); i++, k++) {
        for (size_t j = 0; j < mna.size(); j++) {
            std::cout << mna[i][j] << "\t\t";
        }
        std::cout << "\t\t" << k->first << "\t\t" << rhs[i] << std::endl;
    }
}

void makeGraph(std::map<std::string, std::shared_ptr<Node>> &nodeMap,
               Parser &parser)
{
    for (std::shared_ptr<CircuitElement> circuitElement :
         parser.circuitElements) {
        // Creates/retrieves start node
        std::shared_ptr<Node> nodeStart;
        std::map<std::string, std::shared_ptr<Node>>::iterator startNodeIter =
            nodeMap.find(circuitElement->getNodeA());
        if (startNodeIter != nodeMap.end())
            nodeStart = startNodeIter->second;
        else {
            nodeStart = std::make_shared<Node>();
            nodeStart->name = circuitElement->getNodeA();
            nodeMap[circuitElement->getNodeA()] = nodeStart;
        }

        // Creates/retrieves end node
        std::shared_ptr<Node> nodeEnd;
        std::map<std::string, std::shared_ptr<Node>>::iterator endNodeIter =
            nodeMap.find(circuitElement->getNodeB());
        if (endNodeIter != nodeMap.end())
            nodeEnd = endNodeIter->second;
        else {
            nodeEnd = std::make_shared<Node>();
            nodeEnd->name = circuitElement->getNodeB();
            nodeMap[circuitElement->getNodeB()] = nodeEnd;
        }

        // Edge from start node to end node for nodeStart
        std::shared_ptr<Edge> edgeStartEnd = std::make_shared<Edge>();
        edgeStartEnd->source = nodeStart;
        edgeStartEnd->target = nodeEnd;
        edgeStartEnd->circuitElement = circuitElement;
        nodeStart->edges.push_back(edgeStartEnd);

        // Edge from end node to start node for nodeEnd
        std::shared_ptr<Edge> edgeEndStart = std::make_shared<Edge>();
        edgeEndStart->source = nodeEnd;
        edgeEndStart->target = nodeStart;
        edgeEndStart->circuitElement = circuitElement;
        nodeEnd->edges.push_back(edgeEndStart);
    }
}

void printxX(std::map<std::string, int> &indexMap, Eigen::MatrixXd &X)
{
    std::cout << std::fixed;
    std::cout << std::setprecision(5);

    std::map<std::string, int>::iterator k = indexMap.begin();

    std::cout << "\n";
    for (size_t i = 0; i < indexMap.size(); i++, k++)
        std::cout << k->first << "\t\t" << X(i) << std::endl;
}

static void resetProcessedFlags(
    std::map<std::string, std::shared_ptr<Node>> &nodeMap,
    std::vector<std::shared_ptr<CircuitElement>> &circuitElements)
{
    for (auto &p : nodeMap) {
        if (p.second) p.second->processed = false;
    }

    for (auto &el : circuitElements) {
        if (el) el->setProcessed(false);
    }
}

Eigen::MatrixXd assembleMatrixOnly(
    Parser &parser, std::map<std::string, std::shared_ptr<Node>> &nodeMap,
    std::map<std::string, int> &indexMap, double h)
{
    // Number of unknowns
    int n = static_cast<int>(indexMap.size());

    // Prepare zeroed MNA and RHS (RHS used only as a stamp target here)
    std::vector<std::vector<double>> mna(n, std::vector<double>(n, 0.0));
    std::vector<double> rhs(n, 0.0);

    // 1) Precompute companion params for all elements
    for (auto &el : parser.circuitElements) {
        if (el) el->computeCompanion(h);
    }

    // 2) Reset processed flags so traversal stamps everything
    resetProcessedFlags(nodeMap, parser.circuitElements);

    // 3) Traverse all nodes and stamp transient contributions
    //    Use a traversal that visits every node (skip ground if present)
    for (auto &p : nodeMap) {
        auto nodePtr = p.second;
        if (!nodePtr) continue;
        if (nodePtr->name == "0") continue;
        if (!nodePtr->processed) {
            nodePtr->traverse(indexMap, mna, rhs,
                              true);  // transientMode = true
        }
    }

    // 4) Convert std::vector<std::vector<double>> mna -> Eigen::MatrixXd A
    //    Use the same row-major -> Eigen::Map(...).transpose() approach used
    //    elsewhere.
    std::vector<double> flat;
    flat.reserve(n * n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            flat.push_back(mna[i][j]);
        }
    }

    Eigen::MatrixXd A;
    if (n > 0) {
        A.resize(n, n);
        A = Eigen::MatrixXd::Map(flat.data(), n, n).transpose();
    } else {
        A.resize(0, 0);
    }

    return A;
}

// Compute DC operating point and initialize element states.
// - Builds indexMap/nodeMap using makeIndexMap / makeGraph if they are empty.
// - Assembles DC MNA and RHS (capacitors act as open by existing DC stamps).
// - Solves for x0 and calls element->updateStateFromSolution(x0, indexMap)
//   to initialize v_prev / i_prev / u_prev for transient.
Eigen::VectorXd computeOperatingPoint(
    Parser &parser, std::map<std::string, std::shared_ptr<Node>> &nodeMap,
    std::map<std::string, int> &indexMap)
{
    // Ensure indexMap/nodeMap are populated (caller may pass empty maps)
    if (indexMap.empty()) makeIndexMap(indexMap, parser);
    if (nodeMap.empty()) makeGraph(nodeMap, parser);

    int n = static_cast<int>(indexMap.size());

    // Prepare MNA and RHS
    std::vector<std::vector<double>> mna(n, std::vector<double>(n, 0.0));
    std::vector<double> rhs(n, 0.0);

    // Reset processed flags so traversal stamps everything
    resetProcessedFlags(nodeMap, parser.circuitElements);

    // Traverse all nodes (skip ground) to stamp DC contributions
    for (auto &p : nodeMap) {
        auto nodePtr = p.second;
        if (!nodePtr) continue;
        if (nodePtr->name == "0") continue;
        if (!nodePtr->processed) {
            nodePtr->traverse(indexMap, mna, rhs);  // DC mode (default)
        }
    }

    // Convert mna -> Eigen::MatrixXd and rhs -> Eigen::VectorXd (same pattern
    // used elsewhere)
    Eigen::MatrixXd MNA;
    Eigen::VectorXd RHS;
    if (n > 0) {
        std::vector<double> flat;
        flat.reserve(n * n);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) flat.push_back(mna[i][j]);

        MNA.resize(n, n);
        MNA = Eigen::MatrixXd::Map(flat.data(), n, n).transpose();

        RHS.resize(n);
        RHS = Eigen::MatrixXd::Map(rhs.data(), n, 1);
    } else {
        MNA.resize(0, 0);
        RHS.resize(0);
    }

    // Solve DC system: X = MNA.lu().solve(RHS)
    Eigen::VectorXd X;
    if (n > 0) {
        X = MNA.lu().solve(RHS);
    } else {
        X.resize(0);
    }

    // Initialize element internal states from DC solution

    for (auto &el : parser.circuitElements) {
        if (el) el->updateStateFromSolution(X, indexMap);
    }

    return X;
}

int runTransient(Parser &parser,
                 std::map<std::string, std::shared_ptr<Node>> &nodeMap,
                 std::map<std::string, int> &indexMap, double tFinal, double h,
                 const SolverOptions &options)
{
    std::cout << "Transient solver started: tFinal=" << tFinal << " h=" << h
              << std::endl;

    // Ensure maps are populated (caller may have passed empty maps)
    if (indexMap.empty()) makeIndexMap(indexMap, parser);
    if (nodeMap.empty()) makeGraph(nodeMap, parser);

    // Initialize element states from DC operating point
    Eigen::VectorXd x0 = computeOperatingPoint(parser, nodeMap, indexMap);

    int n = static_cast<int>(indexMap.size());
    if (n == 0) {
        std::cout << "No unknowns in indexMap — nothing to simulate."
                  << std::endl;
        return 0;
    }

    // Newton/TR solver parameters (sane defaults)
    // - maxNewtonIterations: maximum Newton iterations per timestep
    // - tolAbs: absolute tolerance on solution increment
    // - tolRel: relative tolerance on solution increment
    // - tolResidual: tolerance on linear residual ||A_k x - b||
    const int maxNewtonIterations = options.maxNewtonIters;  // allow enough iterations for difficult nonlinearities
    const double tolAbs = 1e-9;       // absolute increment tolerance
    const double tolRel = 1e-6;       // relative increment tolerance
    const double tolResidual = 1e-8;  // residual tolerance (A x - b)
    // Under-relaxation factor for Newton updates (1.0 = no relaxation)
    const double newtonAlpha = options.newtonAlpha;  // sane default damping

    // Adaptive controller knobs (Phase A)
    bool enableAdaptive = options.enableAdaptive; // opt-in: LTE step-doubling
    double atol = options.atol;    // absolute tolerance for LTE
    double rtol = options.rtol;    // relative tolerance for LTE
    double safety = options.safety;   // safety factor for step-size controller
    double fac_min = options.facMin;  // minimum step-change factor
    double fac_max = options.facMax;  // maximum step-change factor
    double h_min = (options.hMin > 0.0) ? options.hMin : (h / 1024.0);  // absolute minimum step
    double h_max = (options.hMax > 0.0) ? options.hMax : h;           // maximum allowed step (initial h)

    // Prepare per-step containers
    std::vector<double> rhs_std(n, 0.0);
    std::vector<std::vector<double>> mna_dummy(n, std::vector<double>(n, 0.0));
    Eigen::VectorXd b(n);
    Eigen::VectorXd x(n);

    // Determine number of steps (ceil-like)
    int steps = static_cast<int>(tFinal / h);
    if (steps * h < tFinal - 1e-12) ++steps;

    // Prepare CSV output
    // Build a vector of unknown names ordered by index (index -> name)
    std::vector<std::string> unknownNames(n);
    for (const auto &p : indexMap) {
        if (p.second >= 0 && p.second < n) unknownNames[p.second] = p.first;
    }

    std::ofstream csv("transient.csv");
    if (!csv) {
        std::cerr << "Warning: Could not open transient.csv for writing. "
                     "Continuing without file output."
                  << std::endl;
    } else {
        // write header
        csv.setf(std::ios::fixed);
        csv << "time";
        for (const auto &name : unknownNames) csv << "," << name;
        csv << "\n";

        // write initial condition at t = 0 (if available)
        csv << std::setprecision(8) << 0.0;
        if (x0.size() == n) {
            for (int i = 0; i < n; ++i) csv << "," << x0(i);
        } else {
            // if x0 not available, write zeros
            for (int i = 0; i < n; ++i) csv << "," << 0.0;
        }
        csv << "\n";
    }

    // Time loop (time-driven to support adaptive timestep retry)
    double current_t = 0.0;
    int step = 0;
    const int maxAdaptiveRetries = options.maxAdaptiveRetries;
    const double minH = (options.hMin > 0.0) ? options.hMin : (h / 1024.0);  // conservative lower bound for h

    // Adaptive timestep state: persist across steps
    double h_current = h;  // current working timestep (may be reduced)
    const double growthFactor = 1.2;  // multiply h_current by this after enough successes
    const int growthThreshold = options.growthSuccessCount;  // number of consecutive successes required to grow
    int consecutiveSuccess = 0;

    // Failure handling policy: choose behavior when max retries are exhausted
    enum class FailurePolicy
    {
        Abort,
        AggressiveFallbackAcceptLast
    };
    const FailurePolicy failurePolicy = FailurePolicy::Abort;
    const int aggressiveAttempts = 3;  // extra aggressive fallback tries

    while (current_t < tFinal - 1e-12) {
        // start each step using the persisted h_current (not always the
        // original h)
        double h_step = h_current;
        // ensure we don't step past tFinal
        if (current_t + h_step > tFinal) h_step = tFinal - current_t;
        double t_local = current_t + h_step;

        // diagnostics file (append) opened per-step to ensure flushed records
        std::ofstream diag(options.diagFile, std::ios::app);
        if (diag) {
            diag.setf(std::ios::fixed);
            diag << std::setprecision(8);
            diag << "--- Diagnostic snapshot: step=" << (step + 1)
                 << " t=" << t_local << " ---\n";
        }

        // prepare a checkpoint of solver/element state before attempting step
        StepCheckpoint cp = createCheckpoint(parser, x0, current_t, h_step);

        bool stepSucceeded = false;
        int attempt = 0;
        for (attempt = 0; attempt <= maxAdaptiveRetries; ++attempt) {
            if (attempt > 0) {
                // restore solver/element state from checkpoint before retry
                if (diag)
                    diag << "Adaptive retry attempt=" << attempt
                         << " restoring checkpoint and reducing h\n";
                restoreCheckpoint(parser, cp, x0);
                // reduce timestep (but not below minH)
                h_step = std::max(h_step * 0.5, minH);
                t_local = current_t + h_step;
                if (diag)
                    diag << "New h=" << h_step << " t_local=" << t_local
                         << "\n";
            }

            // Adaptive path: if enabled use non-committing single-step +
            // two-half-step LTE estimator to accept/reject steps. Otherwise
            // fall back to the existing Newton loop below.
            bool converged = false;
            Eigen::VectorXd xk = x0;  // initial guess: last converged solution
            if (xk.size() != n) xk = Eigen::VectorXd::Zero(n);

            if (enableAdaptive) {
                // 1) Compute one full TR step (non-committing)
                SingleStepResult fullRes = computeSingleStep(
                    parser, nodeMap, indexMap, current_t, h_step, x0, options);

                if (!fullRes.success) {
                    if (diag) {
                        diag << "computeSingleStep failed at t=" << current_t
                             << "; will retry with reduced h\n";
                    }
                    // Treat as failed attempt; reduce timestep on next loop
                    continue;
                }

                // Safety: reject full-step if it produces unphysically large
                // state values even when numerically converged.
                bool fullTooLarge = false;
                for (int i = 0; i < fullRes.x.size(); ++i) {
                    if (!std::isfinite(fullRes.x[i]) ||
                        std::abs(fullRes.x[i]) > options.maxState) {
                        fullTooLarge = true;
                        break;
                    }
                }
                if (fullTooLarge) {
                    if (diag) diag << "computeSingleStep: full-step produced excessive state values; rejecting\n";
                    continue;
                }

                // 2) Compute two half-steps (non-committing)
                SingleStepResult halfRes = computeTwoHalfSteps(
                    parser, nodeMap, indexMap, current_t, h_step, x0, options);

                if (!halfRes.success) {
                    if (diag) {
                        diag << "computeTwoHalfSteps failed at t=" << current_t
                             << "; will retry with reduced h\n";
                    }
                    // Treat as failed attempt; reduce timestep on next loop
                    continue;
                }

                // Safety: reject half-step sequence if it produces unphysically
                // large states.
                bool halfTooLarge = false;
                for (int i = 0; i < halfRes.x.size(); ++i) {
                    if (!std::isfinite(halfRes.x[i]) ||
                        std::abs(halfRes.x[i]) > options.maxState) {
                        halfTooLarge = true;
                        break;
                    }
                }
                if (halfTooLarge) {
                    if (diag) diag << "computeTwoHalfSteps: half-step produced excessive state values; rejecting\n";
                    continue;
                }

                // 3) Estimate LTE and decide accept/reject
                double err_norm =
                    estimateLTE(fullRes.x, halfRes.x, x0, atol, rtol);
                if (!std::isfinite(err_norm)) {
                    if (diag)
                        diag << "Non-finite LTE at t=" << current_t << "\n";
                }

                if (!std::isfinite(err_norm) || err_norm > 1.0) {
                    // Reject step: compute conservative reduction and retry
                    double factor = computeStepFactor(err_norm, safety, fac_min,
                                                      fac_max, 2.0);
                    double h_new =
                        std::max(h_min, std::min(h_max, h_step * factor));
                    if (diag) {
                        diag << "Adaptive reject: err_norm=" << err_norm
                             << " factor=" << factor << " new_h=" << h_new
                             << "\n";
                    }
                    // restore checkpoint state and retry with smaller h
                    restoreCheckpoint(parser, cp, x0);
                    h_step = std::max(h_new, minH);
                    t_local = current_t + h_step;
                    consecutiveSuccess = 0;
                    // continue attempt loop
                    continue;
                }

                // Accept step: commit full-step result, optionally update
                // h_current
                double factor =
                    computeStepFactor(err_norm, safety, fac_min, fac_max, 2.0);
                double h_suggest =
                    std::max(h_min, std::min(h_max, h_step * factor));
                if (diag) {
                    diag << "Adaptive accept: err_norm=" << err_norm
                         << " factor=" << factor
                         << " next_h_suggest=" << h_suggest << "\n";
                }

                // Commit element states using the full-step solution
                for (auto &el : parser.circuitElements) {
                    if (el) el->updateStateFromSolution(fullRes.x, indexMap);
                }

                // Write CSV row
                if (csv) {
                    csv << std::setprecision(8) << t_local;
                    for (int i = 0; i < n; ++i) {
                        double val = 0.0;
                        if (i < fullRes.x.size()) val = fullRes.x(i);
                        csv << "," << val;
                    }
                    csv << "\n";
                }

                // Advance time and accept
                current_t += h_step;
                x0 = fullRes.x;
                stepSucceeded = true;
                ++step;
                h_current = h_suggest;
                ++consecutiveSuccess;

                // Growth policy still applies (use h_current suggestion)
                if (consecutiveSuccess >= growthThreshold) {
                    double grown = std::min(h_current * growthFactor, h_max);
                    if (grown > h_current) {
                        if (diag)
                            diag << "Growing timestep from " << h_current
                                 << " to " << grown << " after "
                                 << consecutiveSuccess << " successes\n";
                        h_current = grown;
                    }
                    consecutiveSuccess = 0;
                }

                // step accepted, break out of attempt loop
                break;
            } else {
                // Newton iterations per time-step (TR-only). Start with
                // previous timestep solution as initial guess.
                if (xk.size() != n) xk = Eigen::VectorXd::Zero(n);

                for (int iter = 0; iter < maxNewtonIterations; ++iter) {
                    // 1) For nonlinear elements compute companions at current
                    // iterate
                    for (auto &el : parser.circuitElements) {
                        if (el) el->computeCompanionIter(h_step, xk, indexMap);
                    }

                    // 2) Assemble A_k and b_k for this Newton iterate
                    std::fill(rhs_std.begin(), rhs_std.end(), 0.0);
                    for (auto &row : mna_dummy)
                        std::fill(row.begin(), row.end(), 0.0);
                    resetProcessedFlags(nodeMap, parser.circuitElements);
                    for (auto &p : nodeMap) {
                        auto nodePtr = p.second;
                        if (!nodePtr) continue;
                        if (nodePtr->name == "0") continue;
                        if (!nodePtr->processed) {
                            nodePtr->traverse(indexMap, mna_dummy, rhs_std,
                                              true);
                        }
                    }

                    // convert mna_dummy -> Eigen A_k
                    std::vector<double> flat;
                    flat.reserve(n * n);
                    for (int i = 0; i < n; ++i) {
                        for (int j = 0; j < n; ++j)
                            flat.push_back(mna_dummy[i][j]);
                    }
                    Eigen::MatrixXd A_k;
                    if (n > 0) {
                        A_k.resize(n, n);
                        A_k =
                            Eigen::MatrixXd::Map(flat.data(), n, n).transpose();
                    } else {
                        A_k.resize(0, 0);
                    }

                    b = Eigen::Map<Eigen::VectorXd>(rhs_std.data(), n);

                    // 3) Solve A_k * x_new = b with LU/solve guards.
                    // If LU reports non-invertible, try small diagonal
                    // regularization attempts before treating the iterate as
                    // failed.
                        // Early sanity checks on assembled matrix/vector. If the
                        // assembly produced non-finite or astronomically large
                        // values, reject this iterate early so the adaptive
                        // controller can reduce the timestep.
                        {
                            bool assemblyOk = true;
                            double maxA = 0.0;
                            double maxB = 0.0;
                            try {
                                if (A_k.size() > 0) maxA = A_k.cwiseAbs().maxCoeff();
                            } catch (...) { assemblyOk = false; }
                            try {
                                if (b.size() > 0) maxB = b.cwiseAbs().maxCoeff();
                            } catch (...) { assemblyOk = false; }
                            if (!assemblyOk || !std::isfinite(maxA) || !std::isfinite(maxB) ||
                                maxA > options.maxState * 1e6 || maxB > options.maxState * 1e6) {
                                if (diag)
                                    diag << "Newton: rejecting iterate due to bad A/b assembly (maxA=" << maxA << " maxB=" << maxB << ")\n";
                                // treat as failed iterate
                                continue;
                            }
                        }
                    Eigen::VectorXd x_new(n);
                    if (A_k.size() > 0) {
                        Eigen::FullPivLU<Eigen::MatrixXd> lu_k;
                        lu_k.compute(A_k);

                        if (!lu_k.isInvertible()) {
                            if (diag)
                                diag << "LU not invertible at iter="
                                     << (iter + 1)
                                     << "; attempting regularization\n";
                            // Try a few regularization magnitudes
                            // (non-accumulating)
                            const double regs[3] = {1e-12, 1e-9, 1e-6};
                            bool solved = false;
                            for (double rv : regs) {
                                Eigen::MatrixXd A_reg = A_k;
                                for (int d = 0; d < n; ++d) A_reg(d, d) += rv;
                                lu_k.compute(A_reg);
                                if (lu_k.isInvertible()) {
                                    x_new = lu_k.solve(b);
                                    // if solve produced finite results, accept
                                    bool ok = true;
                                    for (int i = 0; i < n; ++i)
                                        if (!std::isfinite(x_new[i])) {
                                            ok = false;
                                            break;
                                        }
                                    if (ok) {
                                        solved = true;
                                        break;
                                    }
                                }
                            }
                            if (!solved) {
                                if (diag) {
                                    diag << "LU/regularization failed at iter="
                                         << (iter + 1) << "\n";
                                    double maxA = 0.0;
                                    try {
                                        if (A_k.size() > 0)
                                            maxA = A_k.cwiseAbs().maxCoeff();
                                    } catch (...) {
                                    }
                                    double maxB = 0.0;
                                    try {
                                        if (b.size() > 0)
                                            maxB = b.cwiseAbs().maxCoeff();
                                    } catch (...) {
                                    }
                                    diag << "A_k.maxAbs=" << maxA
                                         << " b.maxAbs=" << maxB << "\n";
                                }
                                // Treat this linear solve as failed: leave xk
                                // unchanged and continue iterating
                                continue;
                            }
                        } else {
                            x_new = lu_k.solve(b);
                        }
                    } else {
                        x_new.setZero();
                    }

                    // 4) Apply under-relaxation with simple backtracking
                    // line-search. Try several alpha values (newtonAlpha,
                    // newtonAlpha/2, ...) and pick the relaxed iterate that
                    // yields the smallest finite residual.
                    double relTol = tolRel * (1.0 + xk.norm());

                    bool xnewFinite = true;
                    for (int i = 0; i < n; ++i) {
                        if (!std::isfinite(x_new[i])) {
                            xnewFinite = false;
                            break;
                        }
                    }
                    if (!xnewFinite) {
                        std::cerr
                            << "Warning: Non-finite values produced by linear "
                               "solve at t="
                            << t_local << ". Treating iterate as failed."
                            << std::endl;
                        if (diag) {
                            diag << "Non-finite x_new produced at iter="
                                 << (iter + 1) << "\n";
                            if (options.diagVerbose) {
                                for (auto &el : parser.circuitElements) {
                                    if (el) el->dumpDiagnostics(diag, xk, indexMap);
                                }
                            }
                            double maxA = 0.0;
                            if (A_k.size() > 0)
                                maxA = A_k.cwiseAbs().maxCoeff();
                            double maxB = 0.0;
                            if (b.size() > 0) maxB = b.cwiseAbs().maxCoeff();
                            diag << "A_k.maxAbs=" << maxA
                                 << " b.maxAbs=" << maxB << "\n";
                        }
                        // Do not accept x_new; leave xk unchanged and continue
                        // iterating.
                        continue;
                    }

                    // Precompute residual at current xk for comparison
                    double prevResidualNorm =
                        std::numeric_limits<double>::infinity();
                    if (A_k.size() > 0) {
                        Eigen::VectorXd r_k = A_k * xk - b;
                        prevResidualNorm = r_k.norm();
                    }

                    const int maxBacktracks = 4;
                    double bestResidual =
                        std::numeric_limits<double>::infinity();
                    Eigen::VectorXd best_x_relaxed = xk;
                    double tryAlpha = newtonAlpha;
                    bool foundFinite = false;

                    for (int bt = 0; bt <= maxBacktracks; ++bt) {
                        Eigen::VectorXd x_relaxed_try =
                            xk + tryAlpha * (x_new - xk);

                        bool xrelFinite = true;
                        for (int i = 0; i < n; ++i) {
                            if (!std::isfinite(x_relaxed_try[i])) {
                                xrelFinite = false;
                                break;
                            }
                        }
                        // Early growth guard: reject relaxed iterate if it
                        // becomes unphysically large compared to previous
                        // iterate.
                        if (xrelFinite) {
                            double maxAbs = 0.0;
                            for (int i = 0; i < n; ++i)
                                maxAbs = std::max(maxAbs, std::abs(x_relaxed_try[i]));
                            const double growthFactorLimit = 1e3;
                            if (maxAbs > options.maxState * 10.0 || x_relaxed_try.norm() > growthFactorLimit * (1.0 + xk.norm())) {
                                if (diag)
                                    diag << "Backtrack: rejecting wildly growing iterate (maxAbs=" << maxAbs << ")\n";
                                xrelFinite = false;
                            }
                        }
                        if (!xrelFinite) {
                            if (diag)
                                diag
                                    << "Backtrack bt=" << bt
                                    << " alpha=" << tryAlpha
                                    << " produced non-finite relaxed iterate\n";
                            tryAlpha *= 0.5;
                            continue;
                        }

                        double residualNorm =
                            std::numeric_limits<double>::infinity();
                        if (A_k.size() > 0) {
                            Eigen::VectorXd r = A_k * x_relaxed_try - b;
                            residualNorm = r.norm();
                        }

                        if (!std::isfinite(residualNorm)) {
                            if (diag)
                                diag << "Backtrack bt=" << bt
                                     << " alpha=" << tryAlpha
                                     << " produced non-finite residual\n";
                            tryAlpha *= 0.5;
                            continue;
                        }

                        // Record the best finite residual
                        if (residualNorm < bestResidual) {
                            bestResidual = residualNorm;
                            best_x_relaxed = x_relaxed_try;
                        }
                        foundFinite = true;

                        // Prefer an alpha that reduces residual compared to
                        // previous iterate
                        if (std::isfinite(prevResidualNorm) &&
                            residualNorm <= prevResidualNorm) {
                            if (diag)
                                diag << "Backtrack: accepted alpha=" << tryAlpha
                                     << " residual=" << residualNorm
                                     << " prevResidual=" << prevResidualNorm
                                     << "\n";
                            break;
                        }

                        // otherwise reduce alpha and try again
                        tryAlpha *= 0.5;
                    }

                    if (!foundFinite) {
                        std::cerr << "Warning: Non-finite values after "
                                     "relaxation/backtracking at t="
                                  << t_local << ". Treating iterate as failed."
                                  << std::endl;
                        if (diag) {
                            diag << "Non-finite x_relaxed produced after "
                                    "backtracking "
                                    "at iter="
                                 << (iter + 1) << "\n";
                        }
                        continue;
                    }

                    Eigen::VectorXd x_relaxed = best_x_relaxed;

                    Eigen::VectorXd delta = x_relaxed - xk;
                    double deltaNorm = delta.norm();

                    // residual r = A_k * x_relaxed - b
                    double residualNorm = 0.0;
                    if (A_k.size() > 0) {
                        Eigen::VectorXd r = A_k * x_relaxed - b;
                        residualNorm = r.norm();
                    }

                    // Diagnostic printing: print detailed Newton progress for
                    // first few timesteps to avoid flooding output for long
                    // simulations.
                    if (step < 5) {
                        std::cout << "  Newton iter " << (iter + 1)
                                  << ": deltaNorm=" << deltaNorm
                                  << " residualNorm=" << residualNorm
                                  << std::endl;
                    }

                    // Convergence if either increment or residual tolerance met
                    // Require finite norms before accepting convergence to
                    // avoid accepting invalid iterates that contain Inf/NaN.
                    if (std::isfinite(deltaNorm) &&
                        std::isfinite(residualNorm) &&
                        (deltaNorm <= tolAbs || deltaNorm <= relTol ||
                         residualNorm <= tolResidual)) {
                        x = x_relaxed;
                        converged = true;
                        if (step < 5) {
                            std::cout << "  Converged at iter " << (iter + 1)
                                      << " deltaNorm=" << deltaNorm
                                      << " residualNorm=" << residualNorm
                                      << " alpha=" << newtonAlpha << std::endl;
                        }
                        break;
                    }

                    // 5) prepare next iterate (accept relaxed update)
                    xk = x_relaxed;
                }

                if (!converged) {
                    std::cerr << "Warning: Newton did not converge in "
                              << maxNewtonIterations
                              << " iterations at t=" << t_local << std::endl;
                    // dump diagnostics for the failed timestep (last iterate
                    // xk)
                        if (diag) {
                        diag << "Newton did not converge in "
                             << maxNewtonIterations << " iters at t=" << t_local
                             << "; dumping diagnostics...\n";
                        if (options.diagVerbose) {
                            for (auto &el : parser.circuitElements) {
                                if (el) el->dumpDiagnostics(diag, xk, indexMap);
                            }
                        }

                        // compute compact A_k/b snapshot for last assembly
                        // Re-assemble a compact A_k/b for the last xk iterate
                        // to show matrix magnitudes (do a quick assemble using
                        // assembleMatrixOnly)
                        try {
                            Eigen::MatrixXd A_snap = assembleMatrixOnly(
                                parser, nodeMap, indexMap, h_step);
                            double maxA = 0.0;
                            if (A_snap.size() > 0)
                                maxA = A_snap.cwiseAbs().maxCoeff();
                            diag << "A_snapshot.maxAbs=" << maxA << "\n";
                        } catch (...) {
                            diag << "A_snapshot assembly failed" << "\n";
                        }
                    }
                    // Accept last iterate
                    x = xk;
                }
            }

            // If converged, consider adaptive acceptance or commit state
            if (converged) {
                bool acceptStep = true;

                if (enableAdaptive) {
                    // Use current Newton solution `x` as the full-step result
                    SingleStepResult halfRes = computeTwoHalfSteps(
                        parser, nodeMap, indexMap, current_t, h_step, x0, options);

                    // Safety: reject half-step result if it produces
                    // excessively large state values
                    bool halfTooLarge = false;
                    if (halfRes.success) {
                        for (int i = 0; i < halfRes.x.size(); ++i) {
                            if (!std::isfinite(halfRes.x[i]) ||
                                std::abs(halfRes.x[i]) > options.maxState) {
                                halfTooLarge = true;
                                break;
                            }
                        }
                    }

                    double err_norm = std::numeric_limits<double>::infinity();
                    if (halfRes.success && !halfTooLarge) {
                        err_norm = estimateLTE(x, halfRes.x, x0, atol, rtol);
                    }

                    if (!halfRes.success || !std::isfinite(err_norm) ||
                        err_norm > 1.0) {
                        // Reject step: compute suggested reduction and retry
                        double factor = computeStepFactor(
                            err_norm, safety, fac_min, fac_max, 2.0);
                        double h_new =
                            std::max(h_min, std::min(h_max, h_step * factor));
                        if (diag) {
                            diag << "Adaptive reject: err_norm=" << err_norm
                                 << " suggested_factor=" << factor
                                 << " new_h=" << h_new << "\n";
                        }
                        // restore checkpoint state and schedule retry with
                        // smaller h
                        restoreCheckpoint(parser, cp, x0);
                        h_step = std::max(h_new, minH);
                        t_local = current_t + h_step;
                        acceptStep = false;
                        consecutiveSuccess = 0;
                        // continue to next attempt (will increment attempt
                        // loop)
                    } else {
                        // accepted by LTE; optionally adjust next h
                        // conservatively
                        double factor = computeStepFactor(
                            err_norm, safety, fac_min, fac_max, 2.0);
                        double h_suggest =
                            std::max(h_min, std::min(h_max, h_step * factor));
                        if (diag) {
                            diag << "Adaptive accept: err_norm=" << err_norm
                                 << " factor=" << factor
                                 << " next_h_suggest=" << h_suggest << "\n";
                        }
                        // commit using x (full step solution)
                        h_current = h_suggest;
                    }
                }

                if (acceptStep) {
                    for (auto &el : parser.circuitElements) {
                        if (el) el->updateStateFromSolution(x, indexMap);
                    }

                    // Write CSV row for this timestep
                    if (csv) {
                        csv << std::setprecision(8) << t_local;
                        for (int i = 0; i < n; ++i) {
                            double val = 0.0;
                            if (i < x.size()) val = x(i);
                            csv << "," << val;
                        }
                        csv << "\n";
                    }

                    // Optional: print progress (first few steps)
                    if (step < 5) {
                        std::cout << "Transient step " << (step + 1) << "/"
                                  << steps << " t=" << t_local << std::endl;
                    }

                    // advance time and accept this step
                    current_t += h_step;
                    x0 = x;  // last converged solution
                    stepSucceeded = true;
                    ++step;

                    // Persist the reduced timestep so the next step starts from
                    // this h_current rather than jumping back to the original
                    // h.
                    ++consecutiveSuccess;

                    // Growth policy: after a number of consecutive successful
                    // steps, increase `h_current` conservatively up to `h_max`.
                    if (consecutiveSuccess >= growthThreshold) {
                        double grown =
                            std::min(h_current * growthFactor, h_max);
                        if (grown > h_current) {
                            if (diag)
                                diag << "Growing timestep from " << h_current
                                     << " to " << grown << " after "
                                     << consecutiveSuccess << " successes\n";
                            h_current = grown;
                        }
                        consecutiveSuccess = 0;
                    }

                    break;
                } else {
                    // rejected by LTE — continue attempt loop (restore done
                    // above)
                    if (diag)
                        diag << "Retrying step with reduced h=" << h_step
                             << "\n";
                    continue;
                }
            } else {
                // iterate did not converge within Newton iterations; try retry
                if (diag)
                    diag << "Step attempt failed at attempt=" << attempt
                         << "\n";
                // reset success counter since a failure occurred
                consecutiveSuccess = 0;
                // continue to next adaptive attempt (restore done above)
                continue;
            }
        }  // end attempt loop

        if (!stepSucceeded) {
            if (failurePolicy == FailurePolicy::Abort) {
                std::cerr << "Fatal: Step failed after " << maxAdaptiveRetries
                          << " retries at t=" << current_t + h_step
                          << ". Aborting simulation." << std::endl;
                if (diag) {
                    diag << "Fatal: Step failed after " << maxAdaptiveRetries
                         << " retries at t=" << current_t + h_step << "\n";
                    if (options.diagVerbose) {
                        for (auto &el : parser.circuitElements) {
                            if (el) el->dumpDiagnostics(diag, x0, indexMap);
                        }
                    }
                }
                // return non-zero to indicate failure to caller
                if (csv) {
                    csv.flush();
                    csv.close();
                }
                std::cerr << "Aborted transient at t=" << current_t
                          << std::endl;
                return 2;
            } else {
                // Aggressive fallback: try a few additional attempts with
                // more aggressive timestep reductions and stronger
                // regularization on the linear solve. If any fallback
                // attempt produces a finite, low-residual solution, accept it.
                if (diag)
                    diag << "Attempting aggressive fallback after "
                         << maxAdaptiveRetries << " retries\n";
                bool fallbackSucceeded = false;
                double h_fallback = h_step;
                const double fallbackRegs[] = {1e-9, 1e-6, 1e-3, 1e-1};
                for (int ag = 0; ag < aggressiveAttempts && !fallbackSucceeded;
                     ++ag) {
                    restoreCheckpoint(parser, cp, x0);
                    h_fallback = std::max(h_fallback * 0.5, minH);
                    double t_fb = current_t + h_fallback;
                    if (diag)
                        diag << "Aggressive attempt " << ag + 1
                             << " h=" << h_fallback << " t=" << t_fb << "\n";

                    // Compute companions at the checkpoint state (x0)
                    for (auto &el : parser.circuitElements) {
                        if (el) el->computeCompanion(h_fallback);
                    }

                    // Assemble A and b for a single linearized solve
                    std::fill(rhs_std.begin(), rhs_std.end(), 0.0);
                    for (auto &row : mna_dummy)
                        std::fill(row.begin(), row.end(), 0.0);
                    resetProcessedFlags(nodeMap, parser.circuitElements);
                    for (auto &p : nodeMap) {
                        auto nodePtr = p.second;
                        if (!nodePtr) continue;
                        if (nodePtr->name == "0") continue;
                        if (!nodePtr->processed)
                            nodePtr->traverse(indexMap, mna_dummy, rhs_std,
                                              true);
                    }

                    // convert mna_dummy -> Eigen A_fb
                    std::vector<double> flat_fb;
                    flat_fb.reserve(n * n);
                    for (int i = 0; i < n; ++i)
                        for (int j = 0; j < n; ++j)
                            flat_fb.push_back(mna_dummy[i][j]);
                    Eigen::MatrixXd A_fb;
                    if (n > 0) {
                        A_fb.resize(n, n);
                        A_fb = Eigen::MatrixXd::Map(flat_fb.data(), n, n)
                                   .transpose();
                    } else {
                        A_fb.resize(0, 0);
                    }
                    Eigen::VectorXd b_fb =
                        Eigen::Map<Eigen::VectorXd>(rhs_std.data(), n);

                    if (A_fb.size() > 0) {
                        Eigen::FullPivLU<Eigen::MatrixXd> lu_fb;
                        lu_fb.compute(A_fb);
                        bool solved = false;
                        // Try stronger regularizations (larger diag additions)
                        for (double rv : fallbackRegs) {
                            Eigen::MatrixXd A_reg = A_fb;
                            for (int d = 0; d < n; ++d) A_reg(d, d) += rv;
                            lu_fb.compute(A_reg);
                            if (!lu_fb.isInvertible()) continue;
                            Eigen::VectorXd x_try = lu_fb.solve(b_fb);
                            bool ok = true;
                            for (int i = 0; i < n; ++i)
                                if (!std::isfinite(x_try[i])) {
                                    ok = false;
                                    break;
                                }
                            if (!ok) continue;
                            // compute residual
                            double resNorm = 0.0;
                            if (A_fb.size() > 0) {
                                Eigen::VectorXd r = A_fb * x_try - b_fb;
                                resNorm = r.norm();
                            }
                            if (std::isfinite(resNorm) &&
                                resNorm <= tolResidual * 100.0) {
                                // accept fallback solution
                                if (diag)
                                    diag << "Aggressive fallback accepted with "
                                            "reg="
                                         << rv << " res=" << resNorm << "\n";
                                // update element states using this solution
                                for (auto &el : parser.circuitElements) {
                                    if (el)
                                        el->updateStateFromSolution(x_try,
                                                                    indexMap);
                                }
                                if (csv) {
                                    csv << std::setprecision(8) << t_fb;
                                    for (int i = 0; i < n; ++i) {
                                        double val = 0.0;
                                        if (i < x_try.size()) val = x_try(i);
                                        csv << "," << val;
                                    }
                                    csv << "\n";
                                }
                                current_t += h_fallback;
                                x0 = x_try;
                                ++step;
                                h_current = h_fallback;
                                ++consecutiveSuccess;
                                fallbackSucceeded = true;
                                solved = true;
                                break;
                            }
                        }
                        if (solved) break;
                    }
                }

                if (!fallbackSucceeded) {
                    std::cerr << "Warning: Aggressive fallback exhausted; "
                                 "accepting last state and advancing."
                              << std::endl;
                    if (diag)
                        diag << "Aggressive fallback exhausted; accepting last "
                                "state and advancing.\n";
                    // Accept last known x0 as the state and advance time
                    // minimally
                    for (auto &el : parser.circuitElements) {
                        if (el) el->updateStateFromSolution(x0, indexMap);
                    }
                    if (csv) {
                        csv << std::setprecision(8) << (current_t + h_step);
                        for (int i = 0; i < n; ++i) {
                            double val = 0.0;
                            if (i < x0.size()) val = x0(i);
                            csv << "," << val;
                        }
                        csv << "\n";
                    }
                    current_t += h_step;
                    ++step;
                }
            }
        }
    }

    if (csv) {
        csv.flush();
        csv.close();
        std::cout << "Transient results written to transient.csv" << std::endl;
    }

    std::cout << "Transient solver finished." << std::endl;
    return 0;
}

int runSolver(int argc, char *argv[], const SolverOptions &options)
{
    // Default filename if not provided as command line argument.
    std::string filename = "circuit.sns";
    if (argc > 1) {
        filename = argv[1];
    }

    // Creates a parser to store the circuit in form of vector
    Parser parser;
    SolverDirectiveType directive = SolverDirectiveType::NONE;
    if (parser.parse(filename, directive) != 0) return 1;

    if (directive == SolverDirectiveType::NONE) {
        std::cerr << "No solver directive provided in the netlist."
                  << " Please provide a valid solver directive." << std::endl;
        return 1;
    }
    if (directive == SolverDirectiveType::OPERATING_POINT) {
        std::cout << "DC Operating Point Analysis Results:" << std::endl;
    }
    if (directive == SolverDirectiveType::TRANSIENT) {
        // Obtain tFinal and h from the parsed .TRAN directive
        double h = parser.tranStep;
        double tFinal = parser.tranStop;

        // Validate parsed transient parameters
        if (h <= 0.0 || tFinal <= 0.0) {
            std::cerr << "Error: Invalid .TRAN parameters. Ensure .TRAN "
                         "<Tstep> <Tstop> "
                      << "are positive numeric values." << std::endl;
            return 1;
        }
        if (h > tFinal) {
            std::cerr << "Error: .TRAN step is greater than stop time."
                      << std::endl;
            return 1;
        }
        std::map<std::string, int> indexMap;
        makeIndexMap(indexMap, parser);

        std::map<std::string, std::shared_ptr<Node>> nodeMap;
        makeGraph(nodeMap, parser);

        for (auto &p : nodeMap) {
            if (p.second) {
                p.second->processed = false;
            }
        }

        for (auto &el : parser.circuitElements) {
            if (el) {
                el->setProcessed(false);
            }
        }

        // Call the transient skeleton (pass options through)
        return runTransient(parser, nodeMap, indexMap, tFinal, h, options);
    }

    // Map to store all nodes' and group_2 elements' index position in MNA and
    // RHS matrix
    std::map<std::string, int> indexMap;
    makeIndexMap(indexMap, parser);

    // Creates MNA and RHS matrix and initializes to 0.0
    int m = int(indexMap.size());
    std::vector<std::vector<double>> mna(m, std::vector<double>(m, 0.0));
    std::vector<double> rhs(m, 0.0);

    // Map to store the graph of the circuit
    std::map<std::string, std::shared_ptr<Node>> nodeMap;
    makeGraph(nodeMap, parser);

    // Picks the first node from nodeMap
    std::map<std::string, std::shared_ptr<Node>>::iterator startNodeIter =
        nodeMap.begin();
    advance(startNodeIter, 1);

    // Traverses the whole graph and populates the MNA and RHS matrices
    if (startNodeIter != nodeMap.end())
        startNodeIter->second->traverse(indexMap, mna, rhs);

    // Code to solve using Eigen Library
    std::vector<double> v;
    for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++) v.push_back(mna[i][j]);

    Eigen::MatrixXd MNA;
    MNA.resize(m, m);
    MNA = Eigen::MatrixXd::Map(&v[0], m, m).transpose();

    Eigen::MatrixXd RHS;
    RHS.resize(m, 1);
    RHS = Eigen::MatrixXd::Map(&rhs[0], m, 1);

    // De-allocating previously allocated
    // memory for solve method to use
    parser.circuitElements.clear();
    for (std::map<std::string, std::shared_ptr<Node>>::iterator i =
             nodeMap.begin();
         i != nodeMap.end(); i++) {
        i->second->edges.clear();
    }
    nodeMap.clear();
    mna.clear();
    rhs.clear();
    v.clear();

    Eigen::MatrixXd X = MNA.lu().solve(RHS);

    printxX(indexMap, X);
    return 0;
}

// (runSolver overload removed; runSolver now accepts SolverOptions directly)
