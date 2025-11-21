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
                 std::map<std::string, int> &indexMap, double tFinal, double h)
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

    // Build (and factorize) A for the current h
    Eigen::MatrixXd A = assembleMatrixOnly(parser, nodeMap, indexMap, h);
    Eigen::PartialPivLU<Eigen::MatrixXd> lu;
    if (A.size() > 0) lu.compute(A);

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

    // Time loop
    for (int step = 0; step < steps; ++step) {
        double t = (step + 1) * h;  // time at end of step (t_{n+1})

        // 1) Update companion parameters (Ieq depends on previous state)
        for (auto &el : parser.circuitElements) {
            if (el) el->computeCompanion(h);
        }

        // 2) Build RHS for this step (do not modify A)
        std::fill(rhs_std.begin(), rhs_std.end(), 0.0);
        for (auto &row : mna_dummy) std::fill(row.begin(), row.end(), 0.0);

        // Ensure stamps will run
        resetProcessedFlags(nodeMap, parser.circuitElements);

        // Traverse and stamp transient contributions into rhs_std (mna_dummy
        // ignored)
        for (auto &p : nodeMap) {
            auto nodePtr = p.second;
            if (!nodePtr) continue;
            if (nodePtr->name == "0") continue;
            if (!nodePtr->processed) {
                nodePtr->traverse(indexMap, mna_dummy, rhs_std, true);
            }
        }

        // 3) Solve A x = b
        b = Eigen::Map<Eigen::VectorXd>(rhs_std.data(), n);
        if (A.size() > 0) {
            x = lu.solve(b);
        } else {
            x.setZero();
        }

        // 4) Update element states from solution
        for (auto &el : parser.circuitElements) {
            if (el) el->updateStateFromSolution(x, indexMap);
        }

        // 5) Write CSV row for this timestep
        if (csv) {
            csv << std::setprecision(8) << t;
            for (int i = 0; i < n; ++i) {
                double val = 0.0;
                if (i < x.size()) val = x(i);
                csv << "," << val;
            }
            csv << "\n";
        }

        // Optional: print progress (first few steps)
        if (step < 5 || step == steps - 1) {
            std::cout << "Transient step " << (step + 1) << "/" << steps
                      << " t=" << t << std::endl;
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

int runSolver(int argc, char *argv[])
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

        // Call the transient skeleton
        return runTransient(parser, nodeMap, indexMap, tFinal, h);
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
