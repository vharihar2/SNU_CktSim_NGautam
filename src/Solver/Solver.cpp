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
 * @file Solver.cpp
 * @brief Implementation of solver entrypoints and MNA assembly helpers.
 *
 * This file contains the concrete implementations for the high-level solver
 * routines declared in `Solver.hpp`. It implements a compact, readable driver
 * for:
 *
 *  - Building name->index maps for unknowns (nodes and group-2 element
 * currents).
 *  - Constructing the node connectivity graph used for traversal-based
 * stamping.
 *  - Assembling MNA matrices for DC and transient analyses (including a helper
 *    to assemble only the matrix for diagnostics).
 *  - Running a fixed-step transient driver that uses trapezoidal-rule
 * companions and Newton iterations with simple regularization and backtracking.
 *
 * Implementation notes (concise):
 *  - API-level documentation (function signatures, parameters, and expected
 *    behavior) lives in `Solver.hpp`. The implementation below contains
 *    focused comments only where algorithmic choices or non-obvious behavior
 *    occur.
 *  - Traversal-based stamping uses a `Node`/`Edge` graph; each node's
 *    `traverse()` method is responsible for stamping incident elements.
 *  - The transient driver defends against ill-conditioned linear solves by:
 *      * performing LU factorization and trying small diagonal regularizers
 * when the factorization reports non-invertibility,
 *      * rejecting non-finite iterates and using under-relaxation/backtracking
 *        to produce stable trial iterates,
 *      * limiting growth of solution norms to protect against runaway iterates.
 *
 * Keep the implementation straightforward and avoid duplicating header-level
 * API docs; update inline notes only when the algorithm or invariants change.
 */

#include "Solver.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

#include "Edge.hpp"

// Forward declarations for functions defined later in this file but used
// by helpers placed earlier (keeps ordering stable across edits).
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

    // Ensure indexMap/nodeMap populated, compute number of unknowns early so
    // we can optionally perform a zero-initialization instead of the DC
    // operating point.
    int n = static_cast<int>(indexMap.size());
    if (n == 0) {
        std::cout << "No unknowns in indexMap — nothing to simulate."
                  << std::endl;
        return 0;
    }

    // Initialize element states either from DC operating point or as zeros
    // depending on the `zeroInit` option.
    Eigen::VectorXd x0;
    if (options.zeroInit) {
        x0 = Eigen::VectorXd::Zero(n);
        // Commit zero state to elements so their internal *_prev values are
        // consistent with a zero-initialized run.
        for (auto &el : parser.circuitElements)
            if (el) el->updateStateFromSolution(x0, indexMap);
    } else {
        x0 = computeOperatingPoint(parser, nodeMap, indexMap);
    }

    // Newton/TR solver parameters
    const int maxNewtonIterations = options.maxNewtonIters;
    const double tolAbs = 1e-9;
    const double tolRel = 1e-6;
    const double tolResidual = 1e-8;
    const double newtonAlpha = options.newtonAlpha;
    const int maxBacktracks = options.maxBacktracks;

    // Prepare per-step containers
    std::vector<double> rhs_std(n, 0.0);
    std::vector<std::vector<double>> mna_dummy(n, std::vector<double>(n, 0.0));
    Eigen::VectorXd b(n);
    Eigen::VectorXd x(n);

    // Determine number of steps (ceil-like) for reporting only
    int steps = static_cast<int>(tFinal / h);
    if (steps * h < tFinal - 1e-12) ++steps;

    // Prepare CSV output
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
        csv.setf(std::ios::fixed);
        csv << "time";
        for (const auto &name : unknownNames) csv << "," << name;
        csv << "\n";
        csv << std::setprecision(8) << 0.0;
        if (x0.size() == n) {
            for (int i = 0; i < n; ++i) csv << "," << x0(i);
        } else {
            for (int i = 0; i < n; ++i) csv << "," << 0.0;
        }
        csv << "\n";
    }

    double current_t = 0.0;
    int step = 0;

    std::cout << "Starting fixed-step TR loop (no adaptivity)" << std::endl;

    while (current_t < tFinal - 1e-12) {
        double h_step = h;
        if (current_t + h_step > tFinal) h_step = tFinal - current_t;
        double t_local = current_t + h_step;

        std::ofstream diag(options.diagFile, std::ios::app);
        if (diag) {
            diag.setf(std::ios::fixed);
            diag << std::setprecision(8);
            diag << "--- Diagnostic snapshot: step=" << (step + 1)
                 << " t=" << t_local << " ---\n";
        }

        // Newton iterations for this fixed timestep
        bool converged = false;
        Eigen::VectorXd xk = x0;
        if (xk.size() != n) xk = Eigen::VectorXd::Zero(n);

        for (int iter = 0; iter < maxNewtonIterations; ++iter) {
            // 1) Per-element companion updates at current iterate
            for (auto &el : parser.circuitElements) {
                if (el) el->computeCompanionIter(h_step, xk, indexMap);
            }

            // 2) Assemble A_k and b_k
            std::fill(rhs_std.begin(), rhs_std.end(), 0.0);
            for (auto &row : mna_dummy) std::fill(row.begin(), row.end(), 0.0);
            resetProcessedFlags(nodeMap, parser.circuitElements);
            for (auto &p : nodeMap) {
                auto nodePtr = p.second;
                if (!nodePtr) continue;
                if (nodePtr->name == "0") continue;
                if (!nodePtr->processed)
                    nodePtr->traverse(indexMap, mna_dummy, rhs_std, true);
            }

            // convert mna_dummy -> Eigen A_k
            std::vector<double> flat;
            flat.reserve(n * n);
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j) flat.push_back(mna_dummy[i][j]);

            Eigen::MatrixXd A_k;
            if (n > 0) {
                A_k.resize(n, n);
                A_k = Eigen::MatrixXd::Map(flat.data(), n, n).transpose();
            } else {
                A_k.resize(0, 0);
            }

            b = Eigen::Map<Eigen::VectorXd>(rhs_std.data(), n);

            // Early sanity checks on assembly
            bool assemblyOk = true;
            double maxA = 0.0, maxB = 0.0;
            try {
                if (A_k.size() > 0) maxA = A_k.cwiseAbs().maxCoeff();
            } catch (...) {
                assemblyOk = false;
            }
            try {
                if (b.size() > 0) maxB = b.cwiseAbs().maxCoeff();
            } catch (...) {
                assemblyOk = false;
            }

            if (!assemblyOk || !std::isfinite(maxA) || !std::isfinite(maxB) ||
                maxA > options.maxState * 1e6 ||
                maxB > options.maxState * 1e6) {
                if (diag)
                    diag << "Newton: rejecting iterate due to bad A/b assembly "
                            "(maxA="
                         << maxA << " maxB=" << maxB << ")\n";
                // treat as failed iterate: skip to next Newton iteration (no
                // adaptivity)
                continue;
            }

            // 3) Solve linear system with regularization fallback
            Eigen::VectorXd x_new(n);
            if (A_k.size() > 0) {
                Eigen::FullPivLU<Eigen::MatrixXd> lu_k;
                lu_k.compute(A_k);
                if (!lu_k.isInvertible()) {
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
                        if (diag)
                            diag << "LU/regularization failed at iter="
                                 << (iter + 1) << "\n";
                        continue;
                    }
                } else {
                    x_new = lu_k.solve(b);
                }
            } else {
                x_new.setZero();
            }

            // 4) Backtracking / under-relaxation to obtain finite, stable
            // iterate
            bool xnewFinite = true;
            for (int i = 0; i < n; ++i)
                if (!std::isfinite(x_new[i])) {
                    xnewFinite = false;
                    break;
                }
            if (!xnewFinite) {
                if (diag)
                    diag << "Non-finite x_new at iter=" << (iter + 1) << "\n";
                continue;
            }

            double prevResidualNorm = std::numeric_limits<double>::infinity();
            if (A_k.size() > 0) {
                Eigen::VectorXd r_k = A_k * xk - b;
                prevResidualNorm = r_k.norm();
            }

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
                if (!xrelFinite) {
                    tryAlpha *= 0.5;
                    continue;
                }

                // growth guard
                double maxAbs = x_relaxed_try.cwiseAbs().maxCoeff();
                if (!std::isfinite(maxAbs) ||
                    maxAbs > options.maxState * 10.0 ||
                    x_relaxed_try.norm() > 1e6 * (1.0 + xk.norm())) {
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

            if (!foundFinite) {
                if (diag)
                    diag << "No finite relaxed iterate at iter=" << (iter + 1)
                         << "\n";
                continue;
            }

            Eigen::VectorXd x_relaxed = best_x_relaxed;
            Eigen::VectorXd delta = x_relaxed - xk;
            double deltaNorm = delta.norm();
            double residualNorm = 0.0;
            if (A_k.size() > 0) {
                Eigen::VectorXd r = A_k * x_relaxed - b;
                residualNorm = r.norm();
            }

            if (step < 5) {
                std::cout << "  Newton iter " << (iter + 1)
                          << ": deltaNorm=" << deltaNorm
                          << " residualNorm=" << residualNorm << std::endl;
            }

            if (std::isfinite(deltaNorm) && std::isfinite(residualNorm) &&
                (deltaNorm <= tolAbs ||
                 deltaNorm <= tolRel * (1.0 + xk.norm()) ||
                 residualNorm <= tolResidual)) {
                x = x_relaxed;
                converged = true;
                if (step < 5)
                    std::cout << "  Converged at iter " << (iter + 1)
                              << " deltaNorm=" << deltaNorm
                              << " residualNorm=" << residualNorm << std::endl;
                break;
            }

            xk = x_relaxed;
        }  // end Newton iter loop

        if (!converged) {
            std::cerr << "Warning: Newton did not converge in "
                      << maxNewtonIterations << " iterations at t=" << t_local
                      << std::endl;
            if (diag) {
                diag << "Newton did not converge in " << maxNewtonIterations
                     << " iters at t=" << t_local
                     << "; dumping diagnostics...\n";
                if (options.diagVerbose) {
                    for (auto &el : parser.circuitElements)
                        if (el) el->dumpDiagnostics(diag, xk, indexMap);
                }
                try {
                    Eigen::MatrixXd A_snap =
                        assembleMatrixOnly(parser, nodeMap, indexMap, h_step);
                    double maxA = 0.0;
                    if (A_snap.size() > 0) maxA = A_snap.cwiseAbs().maxCoeff();
                    diag << "A_snapshot.maxAbs=" << maxA << "\n";
                } catch (...) {
                    diag << "A_snapshot assembly failed\n";
                }
            }
            // Accept last iterate even if not fully converged to make progress
            x = xk;
        }

        // Commit state and write output
        for (auto &el : parser.circuitElements)
            if (el) el->updateStateFromSolution(x, indexMap);

        if (csv) {
            csv << std::setprecision(8) << t_local;
            for (int i = 0; i < n; ++i) {
                double val = 0.0;
                if (i < x.size()) val = x(i);
                csv << "," << val;
            }
            csv << "\n";
        }

        if (step < 5)
            std::cout << "Transient step " << (step + 1) << "/" << steps
                      << " t=" << t_local << std::endl;

        current_t += h_step;
        x0 = x;
        ++step;
    }  // end time loop

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

    // Parse the netlist
    Parser parser;
    SolverDirectiveType directive = SolverDirectiveType::NONE;
    if (parser.parse(filename, directive) != 0) {
        std::cerr << "Error: Failed to parse file: " << filename << std::endl;
        return 1;
    }

    if (directive == SolverDirectiveType::NONE) {
        std::cerr << "No solver directive provided in the netlist."
                  << " Please provide a valid solver directive." << std::endl;
        return 1;
    }

    if (directive == SolverDirectiveType::OPERATING_POINT) {
        // Compute operating point and print results
        std::map<std::string, int> indexMap;
        makeIndexMap(indexMap, parser);

        std::map<std::string, std::shared_ptr<Node>> nodeMap;
        makeGraph(nodeMap, parser);

        Eigen::VectorXd x0 = computeOperatingPoint(parser, nodeMap, indexMap);
        std::cout << "DC Operating Point Analysis Results:" << std::endl;
        if (x0.size() > 0) {
            Eigen::MatrixXd X(x0.size(), 1);
            for (int i = 0; i < x0.size(); ++i) X(i, 0) = x0(i);
            printxX(indexMap, X);
        } else {
            std::cout << "No unknowns found in operating point analysis."
                      << std::endl;
        }
        return 0;
    }

    if (directive == SolverDirectiveType::TRANSIENT) {
        // Obtain tFinal and h from the parsed .TRAN directive
        double h = parser.tranStep;
        double tFinal = parser.tranStop;

        // Validate parsed transient parameters
        if (h <= 0.0 || tFinal <= 0.0) {
            std::cerr << "Error: Invalid .TRAN parameters. Ensure .TRAN "
                         "<Tstep> <Tstop> are positive numeric values."
                      << std::endl;
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

        // reset processed flags on nodes and elements for a clean assembly
        for (auto &p : nodeMap)
            if (p.second) p.second->processed = false;
        for (auto &el : parser.circuitElements)
            if (el) el->setProcessed(false);

        // Run the transient solver (fixed-step TR)
        return runTransient(parser, nodeMap, indexMap, tFinal, h, options);
    }

    // Fallback / default DC solve path (if directive wasn't TRANSIENT or O.P.)
    std::map<std::string, int> indexMap;
    makeIndexMap(indexMap, parser);

    int m = int(indexMap.size());
    std::vector<std::vector<double>> mna(m, std::vector<double>(m, 0.0));
    std::vector<double> rhs(m, 0.0);

    std::map<std::string, std::shared_ptr<Node>> nodeMap;
    makeGraph(nodeMap, parser);

    // Pick a start node (skip the first to avoid ground if present)
    std::map<std::string, std::shared_ptr<Node>>::iterator startNodeIter =
        nodeMap.begin();
    if (startNodeIter != nodeMap.end()) advance(startNodeIter, 1);

    // Traverse the graph to populate MNA and RHS
    if (startNodeIter != nodeMap.end())
        startNodeIter->second->traverse(indexMap, mna, rhs);

    // Convert to Eigen matrices and solve
    std::vector<double> v;
    v.reserve(m * m);
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < m; ++j) v.push_back(mna[i][j]);

    Eigen::MatrixXd MNA;
    MNA.resize(m, m);
    if (m > 0)
        MNA = Eigen::MatrixXd::Map(&v[0], m, m).transpose();
    else
        MNA.resize(0, 0);

    Eigen::MatrixXd RHS;
    RHS.resize(m, 1);
    if (m > 0)
        RHS = Eigen::MatrixXd::Map(&rhs[0], m, 1);
    else
        RHS.resize(0, 1);

    // Clean up parser structures we no longer need
    parser.circuitElements.clear();
    for (auto it = nodeMap.begin(); it != nodeMap.end(); ++it)
        it->second->edges.clear();
    nodeMap.clear();
    mna.clear();
    rhs.clear();
    v.clear();

    if (MNA.size() == 0) {
        std::cerr << "Nothing to solve (empty MNA)" << std::endl;
        return 1;
    }

    Eigen::MatrixXd X = MNA.lu().solve(RHS);
    printxX(indexMap, X);
    return 0;
}
