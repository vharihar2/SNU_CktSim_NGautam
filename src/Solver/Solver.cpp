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

#include <iomanip>
#include <iostream>
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

int runSolver(int argc, char *argv[])
{
    // Default filename if not provided as command line argument.
    std::string filename = "circuit.sns";
    if (argc > 1) {
        filename = argv[1];
    }

    // Creates a parser to store the circuit in form of vector
    Parser parser;
    if (parser.parse(filename) != 0) return 1;

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
