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
 * @file Solver.hpp
 * @brief High-level solver API and helpers for MNA assembly and execution.
 *
 * This header exposes the primary entrypoints and utility functions used by the
 * simulator to:
 *  - Build index maps for nodes and group-2 elements,
 *  - Construct the graph of nodes/elements for traversal-based stamping,
 *  - Assemble MNA matrices for DC and transient analysis, and
 *  - Run full solver workflows (operating-point and transient).
 *
 * The implementation details and algorithmic comments are kept in the
 * corresponding implementation file (`Solver.cpp`) while this header documents
 * the public API used by clients and tests.
 */
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "../lib/external/Eigen/Dense"
#include "Node.hpp"
#include "Parser.hpp"
#include "SolverOptions.hpp"

/**
 * @enum SolverDirectiveType
 * @brief Describes the simulation mode requested in a parsed netlist.
 *
 * Parser::parse inspects the netlist for directives (e.g., `.OP`, `.TRAN`)
 * and sets the directive accordingly. Callers use this enum to decide which
 * solver routine to run.
 */
enum class SolverDirectiveType
{
    NONE,            /**< No directive found in the netlist. */
    OPERATING_POINT, /**< Compute DC operating point (.OP). */
    TRANSIENT        /**< Run transient simulation (.TRAN). */
};

/**
 * @brief Build a name -> index map for all unknowns (nodes and G2 elements).
 *
 * The solver represents unknowns (node voltages and group-2 element currents)
 * as a contiguous index space used to index the MNA matrix and RHS vector.
 * This helper fills `indexMap` with an index for each name in
 * `parser.nodes_group2` except ground ("0"). The ordering is deterministic
 * based on the iteration order of the set (which is lexicographic for
 * std::set).
 *
 * @param[out] indexMap Mapping from node or element name -> matrix index.
 * @param[in] parser Parser instance containing `nodes_group2` and parsed
 * elements.
 */
void makeIndexMap(std::map<std::string, int> &indexMap, Parser &parser);

/**
 * @brief Print the MNA matrix and RHS vector (debug helper).
 *
 * Produces a readable tabular dump to stdout showing each matrix row, the
 * corresponding variable name (obtained by iterating through `indexMap`) and
 * the RHS entry. This is intended for debugging and diagnostic purposes.
 *
 * @param[in] mna MNA matrix represented as a row-major std::vector-of-vectors.
 * @param[in] indexMap Mapping from variable names to matrix indices (used to
 *                     print the variable name for each row).
 * @param[in] rhs RHS vector corresponding to the matrix rows.
 */
void printMNAandRHS(std::vector<std::vector<double>> &mna,
                    std::map<std::string, int> &indexMap,
                    std::vector<double> &rhs);

/**
 * @brief Build the node connectivity graph used for traversal stamping.
 *
 * The parser produces a flat list of `CircuitElement` objects. `makeGraph`
 * constructs a graph of `Node` objects (one per node name) and populates
 * each node's outgoing/incoming `Edge` list with shared pointers to elements
 * that connect the nodes. The graph is used by node-based traversal routines
 * that stamp element contributions into the MNA matrix.
 *
 * @param[out] nodeMap Map from node name to shared pointer of `Node`.
 * @param[in] parser Parser instance providing `circuitElements`.
 */
void makeGraph(std::map<std::string, std::shared_ptr<Node>> &nodeMap,
               Parser &parser);

/**
 * @brief Print variable names and their numeric values from an Eigen vector.
 *
 * @param[in] indexMap Mapping of variable name to index (iteration order is
 *                     used to determine name ordering).
 * @param[in] X Solution vector (nx1) containing variable values.
 */
void printxX(std::map<std::string, int> &indexMap, Eigen::MatrixXd &X);

/**
 * @brief Assemble and return only the MNA matrix for the current parser state.
 *
 * This helper performs the transient companion precomputation and node graph
 * traversal stamping (transient mode) to create the square MNA matrix A
 * corresponding to the current time-step companion parameters. The returned
 * matrix is an Eigen::MatrixXd suitable for solvers or diagnostics.
 *
 * Note: This routine does not return the RHS; use the transient assembly
 * path in `runTransient` to obtain the matched RHS vector.
 *
 * @param[in] parser Parsed netlist and element list.
 * @param[in] nodeMap Pre-built node graph used for traversal stamping.
 * @param[in] indexMap Name -> index map used for matrix sizing and stamping.
 * @param[in] h Time-step used for companion computation (transient stamps).
 * @return Eigen::MatrixXd Assembled square MNA matrix (may be 0x0 if no
 * unknowns).
 */
Eigen::MatrixXd assembleMatrixOnly(
    Parser &parser, std::map<std::string, std::shared_ptr<Node>> &nodeMap,
    std::map<std::string, int> &indexMap, double h);

/**
 * @brief Compute the DC operating point for the parsed circuit.
 *
 * The routine builds the MNA system for DC (capacitors behave as open in DC),
 * solves the linear system, and then calls `updateStateFromSolution` on each
 * element so that transient initial states (v_prev, i_prev, ...) are set
 * consistently for subsequent transient runs.
 *
 * @param[in] parser Parsed netlist and element list.
 * @param[in] nodeMap Pre-built node graph (if empty the function will call
 *                    `makeGraph` to construct it).
 * @param[in] indexMap Name -> index map (if empty the function will call
 *                     `makeIndexMap` to construct it).
 * @return Eigen::VectorXd Solution vector X containing the computed unknowns.
 */
Eigen::VectorXd computeOperatingPoint(
    Parser &parser, std::map<std::string, std::shared_ptr<Node>> &nodeMap,
    std::map<std::string, int> &indexMap);

/**
 * @brief Run a fixed-step transient simulation using trapezoidal rule + Newton.
 *
 * This routine implements a fixed-step transient algorithm:
 *  - Initialize element internal states either from the DC operating point or
 *    by zero-initialization depending on `options`.
 *  - For each time-step, perform Newton iterations:
 *      - Update per-element companion linearizations around current iterate.
 *      - Assemble the linearized MNA matrix A_k and residual b_k.
 *      - Solve the linear system with LU + small-regularization fallback.
 *      - Perform under-relaxation/backtracking to ensure finite iterates and
 *        residual reduction.
 *      - Commit the converged iterate and update element states.
 *  - Optionally produce a CSV trace (`transient.csv`) and write diagnostics.
 *
 * The implementation is designed to be robust: it clamps/regularizes poorly
 * conditioned linear systems, rejects iterates with non-finite values, and
 * limits growth of solution vectors to defend against blow-up.
 *
 * @param[in,out] parser Parsed circuit and owned element objects (element
 *                       companion/state methods will be invoked).
 * @param[in,out] nodeMap Node graph used for stamping (constructed by caller
 *                       if empty).
 * @param[in] indexMap Name -> index mapping for unknowns.
 * @param[in] tFinal Final simulation time (seconds).
 * @param[in] h Time step size (seconds).
 * @param[in] options SolverOptions struct controlling tolerances, diagnostics,
 *                    and behavior (default-constructed options are used if
 *                    not supplied).
 * @return 0 on successful completion, non-zero on error (e.g., invalid inputs).
 */
int runTransient(Parser &parser,
                 std::map<std::string, std::shared_ptr<Node>> &nodeMap,
                 std::map<std::string, int> &indexMap, double tFinal, double h,
                 const SolverOptions &options = SolverOptions());

/**
 * @brief Run the top-level solver workflow for a netlist file.
 *
 * This convenience entrypoint performs the overall workflow used by the
 * command-line driver:
 *  - Determine the input filename (from argv or default).
 *  - Parse the netlist and inspect the solver directive (.OP / .TRAN).
 *  - Build index/node maps and dispatch to `computeOperatingPoint` or
 *    `runTransient` depending on the directive.
 *
 * @param[in] argc Count of command-line arguments.
 * @param[in] argv Null-terminated array of argument strings (argv[0] unused).
 * @param[in] options Optional SolverOptions controlling solver behavior.
 * @return 0 on success, non-zero on fatal error (parse/validation failures).
 */
int runSolver(int argc, char *argv[],
              const SolverOptions &options = SolverOptions());

/* End of Solver.hpp */
