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
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "../lib/external/Eigen/Dense"
#include "Node.hpp"
#include "Parser.hpp"

/*
 * @file Solver.hpp
 *
 * @brief Contains the definition of the main functions
 */

/**
 * @brief Creates a map of nodes and group_2 elements to their index positions
 * in MNA and RHS matrices.
 *
 * @param indexMap Map storing the name of the node/element and its
 * corresponding index in the MNA and RHS matrices.
 * @param parser Reference to the created Parser object.
 */
void makeIndexMap(std::map<std::string, int> &indexMap, Parser &parser);

/**
 * @brief Prints the MNA matrix and RHS vector.
 *
 * @param mna The Modified Nodal Analysis (MNA) matrix.
 * @param indexMap The index map created by makeIndexMap.
 * @param rhs The right-hand side vector.
 */
void printMNAandRHS(std::vector<std::vector<double>> &mna,
                    std::map<std::string, int> &indexMap,
                    std::vector<double> &rhs);

/**
 * @brief Creates the circuit graph from the vector of elements for traversal.
 *
 * @param nodeMap Map from node names to shared pointers to Node objects.
 * @param parser Reference to the Parser object.
 */
void makeGraph(std::map<std::string, std::shared_ptr<Node>> &nodeMap,
               Parser &parser);

/**
 * @brief Prints the solution vector X along with unknown variables.
 *
 * @param indexMap Map from variable names to indices.
 * @param X Solution vector (Eigen::MatrixXd).
 */
void printxX(std::map<std::string, int> &indexMap, Eigen::MatrixXd &X);

/**
 * @brief Runs the circuit solver.
 *
 * This function contains the entire workflow to run the solver, including
 * parsing, matrix construction, and solution.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return 0 if successful, else 1.
 */
int runSolver(int argc, char *argv[]);
