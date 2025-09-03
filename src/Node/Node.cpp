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
 * @file Node.cpp
 *
 * @brief Contains the implementation of the Node class
 */

#include "Node.hpp"

#include "CircuitElement.hpp"

void Node::traverse(std::map<std::string, int> &indexMap,
                    std::vector<std::vector<double>> &mna,
                    std::vector<double> &rhs)
{
    // Skip ground node and already processed nodes
    if (name == "0" || processed) return;
    processed = true;

    // Process each edge (circuit element) connected to this node
    for (const auto &edge : edges) {
        if (edge->circuitElement->processed) continue;
        edge->circuitElement->processed = true;

        switch (edge->circuitElement->type) {
            case ElementType::R:
                handleResistor(edge, indexMap, mna, rhs);
                break;
            case ElementType::C:
                handleCapacitor(edge, indexMap, mna, rhs);
                break;
            case ElementType::L:
                handleInductor(edge, indexMap, mna, rhs);
                break;
            case ElementType::I:
                handleCurrentSource(edge, indexMap, mna, rhs);
                break;
            case ElementType::V:
                handleVoltageSource(edge, indexMap, mna, rhs);
                break;
            case ElementType::Vc:
                handleDepVoltageSource(edge, indexMap, mna, rhs);
                break;
            case ElementType::Ic:
                handleDepCurrentSource(edge, indexMap, mna, rhs);
                break;
            // Add other element types as needed
            default:
                // Optionally handle unknown types
                break;
        }
    }

    // Recursively traverse connected nodes
    for (const auto &edge : edges) {
        edge->target->traverse(indexMap, mna, rhs);
    }
}

void Node::handleResistor(const std::shared_ptr<Edge> &edge,
                          std::map<std::string, int> &indexMap,
                          std::vector<std::vector<double>> &mna,
                          std::vector<double> &rhs)
{
    // Group 1: Standard resistor between two nodes
    if (edge->circuitElement->group == Group::G1) {
        if (edge->circuitElement->nodeA == "0") {
            int vminus = indexMap[edge->circuitElement->nodeB];
            mna[vminus][vminus] += 1.0 / edge->circuitElement->value;
        } else if (edge->circuitElement->nodeB == "0") {
            int vplus = indexMap[edge->circuitElement->nodeA];
            mna[vplus][vplus] += 1.0 / edge->circuitElement->value;
        } else {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int vminus = indexMap[edge->circuitElement->nodeB];
            double conductance = 1.0 / edge->circuitElement->value;
            mna[vplus][vplus] += conductance;
            mna[vplus][vminus] += -conductance;
            mna[vminus][vplus] += -conductance;
            mna[vminus][vminus] += conductance;
        }
    }
    // Group 2: Resistor current as variable
    else {
        if (edge->circuitElement->nodeA == "0") {
            int vminus = indexMap[edge->circuitElement->nodeB];
            int i = indexMap[edge->circuitElement->name];
            mna[vminus][i] += -1.0;
            mna[i][vminus] += -1.0;
            mna[i][i] += -edge->circuitElement->value;
        } else if (edge->circuitElement->nodeB == "0") {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int i = indexMap[edge->circuitElement->name];
            mna[vplus][i] += 1.0;
            mna[i][i] += -edge->circuitElement->value;
            mna[i][vplus] += 1.0;
        } else {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int vminus = indexMap[edge->circuitElement->nodeB];
            int i = indexMap[edge->circuitElement->name];
            mna[vplus][i] += 1.0;
            mna[vminus][i] += -1.0;
            mna[i][vplus] += 1.0;
            mna[i][vminus] += -1.0;
            mna[i][i] += -edge->circuitElement->value;
        }
    }
}

void Node::handleCapacitor(const std::shared_ptr<Edge> &edge,
                           std::map<std::string, int> &indexMap,
                           std::vector<std::vector<double>> &mna,
                           std::vector<double> &rhs)
{
    // Group 1: Capacitor is open circuit in DC, so do nothing
    if (edge->circuitElement->group == Group::G1) {
        // No contribution to MNA or RHS
    }
    // Group 2: For DC, typically also open, but if you need a placeholder:
    else {
        int i = indexMap[edge->circuitElement->name];
        mna[i][i] += 1.0;  // Placeholder, can be omitted for pure DC
    }
}

void Node::handleInductor(const std::shared_ptr<Edge> &edge,
                          std::map<std::string, int> &indexMap,
                          std::vector<std::vector<double>> &mna,
                          std::vector<double> &rhs)
{
    // Inductor (always Group 2)
    if (edge->circuitElement->nodeA.compare("0") == 0) {
        int vminus = indexMap[edge->circuitElement->nodeB];
        int i = indexMap[edge->circuitElement->name];
        mna[vminus][i] += 1.0;
        mna[i][vminus] += 1.0;
    } else if (edge->circuitElement->nodeB.compare("0") == 0) {
        int vplus = indexMap[edge->circuitElement->nodeA];
        int i = indexMap[edge->circuitElement->name];
        mna[vplus][i] += 1.0;
        mna[i][vplus] += 1.0;
    } else {
        int vplus = indexMap[edge->circuitElement->nodeA];
        int vminus = indexMap[edge->circuitElement->nodeB];
        int i = indexMap[edge->circuitElement->name];
        mna[vplus][i] += 1.0;
        mna[vminus][i] += -1.0;
        mna[i][vplus] += 1.0;
        mna[i][vminus] += -1.0;
    }
}

void Node::handleCurrentSource(const std::shared_ptr<Edge> &edge,
                               std::map<std::string, int> &indexMap,
                               std::vector<std::vector<double>> &mna,
                               std::vector<double> &rhs)
{
    if (edge->circuitElement->group == Group::G1) {
        if (edge->circuitElement->nodeA.compare("0") == 0) {
            int vminus = indexMap[edge->circuitElement->nodeB];
            rhs[vminus] += edge->circuitElement->value;
        } else if (edge->circuitElement->nodeB.compare("0") == 0) {
            int vplus = indexMap[edge->circuitElement->nodeA];
            rhs[vplus] += -edge->circuitElement->value;
        } else {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int vminus = indexMap[edge->circuitElement->nodeB];
            rhs[vplus] += -edge->circuitElement->value;
            rhs[vminus] += edge->circuitElement->value;
        }
    } else {
        if (edge->circuitElement->nodeA.compare("0") == 0) {
            int vminus = indexMap[edge->circuitElement->nodeB];
            int i = indexMap[edge->circuitElement->name];
            mna[vminus][i] = -1.0;
            mna[i][i] = +1.0;
            rhs[i] = edge->circuitElement->value;
        } else if (edge->circuitElement->nodeB.compare("0") == 0) {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int i = indexMap[edge->circuitElement->name];
            mna[vplus][i] = +1.0;
            mna[i][i] = +1.0;
            rhs[i] = edge->circuitElement->value;
        } else {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int vminus = indexMap[edge->circuitElement->nodeB];
            int i = indexMap[edge->circuitElement->name];
            mna[vplus][i] = +1.0;
            mna[vminus][i] = -1.0;
            mna[i][i] = +1.0;
            rhs[i] = edge->circuitElement->value;
        }
    }
}

void Node::handleVoltageSource(const std::shared_ptr<Edge> &edge,
                               std::map<std::string, int> &indexMap,
                               std::vector<std::vector<double>> &mna,
                               std::vector<double> &rhs)
{
    // Independent Voltage Source (always Group 2)
    if (edge->circuitElement->nodeA.compare("0") == 0) {
        int vminus = indexMap[edge->circuitElement->nodeB];
        int i = indexMap[edge->circuitElement->name];
        mna[vminus][i] += -1.0;
        mna[i][vminus] += -1.0;
        rhs[i] += edge->circuitElement->value;
    } else if (edge->circuitElement->nodeB.compare("0") == 0) {
        int vplus = indexMap[edge->circuitElement->nodeA];
        int i = indexMap[edge->circuitElement->name];
        mna[vplus][i] += 1.0;
        mna[i][vplus] += 1.0;
        rhs[i] += edge->circuitElement->value;
    } else {
        int vplus = indexMap[edge->circuitElement->nodeA];
        int vminus = indexMap[edge->circuitElement->nodeB];
        int i = indexMap[edge->circuitElement->name];
        mna[vplus][i] += 1.0;
        mna[vminus][i] += -1.0;
        mna[i][vplus] += 1.0;
        mna[i][vminus] += -1.0;
        rhs[i] += edge->circuitElement->value;
    }
}

void Node::handleDepVoltageSource(const std::shared_ptr<Edge> &edge,
                                  std::map<std::string, int> &indexMap,
                                  std::vector<std::vector<double>> &mna,
                                  std::vector<double> &rhs)
{
    // Dependant Voltage Source (always Group 2)
    // Current Controlled Voltage Source (CCVS)
    if (edge->circuitElement->controlling_variable == ControlVariable::i) {
        if (edge->circuitElement->nodeA.compare("0") == 0) {
            int vminus = indexMap[edge->circuitElement->nodeB];
            int is = indexMap[edge->circuitElement->name];
            int ix = indexMap[edge->circuitElement->controlling_element->name];
            mna[vminus][is] += -1.0;
            mna[is][vminus] += -1.0;
            mna[is][ix] += -edge->circuitElement->value;
        } else if (edge->circuitElement->nodeB.compare("0") == 0) {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int is = indexMap[edge->circuitElement->name];
            int ix = indexMap[edge->circuitElement->controlling_element->name];
            mna[vplus][is] += 1.0;
            mna[is][vplus] += 1.0;
            mna[is][ix] += -edge->circuitElement->value;
        } else {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int vminus = indexMap[edge->circuitElement->nodeB];
            int is = indexMap[edge->circuitElement->name];
            int ix = indexMap[edge->circuitElement->controlling_element->name];
            mna[vplus][is] += 1.0;
            mna[vminus][is] += -1.0;
            mna[is][vplus] += 1.0;
            mna[is][vminus] += -1.0;
            mna[is][ix] += -edge->circuitElement->value;
        }
    }
    // Voltage Controlled Voltage Source (VCVS)
    else {
        if (edge->circuitElement->nodeA.compare("0") == 0) {
            int vminus = indexMap[edge->circuitElement->nodeB];
            // When controlling_element's source node is connected to ground
            if (edge->circuitElement->controlling_element->nodeA == "0") {
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                int i = indexMap[edge->circuitElement->name];
                mna[vminus][i] += -1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxminus] += edge->circuitElement->value;
            }
            // When controlling_element's target node is connected to ground
            else if (edge->circuitElement->controlling_element->nodeB == "0") {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                int i = indexMap[edge->circuitElement->name];
                mna[vminus][i] += -1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -edge->circuitElement->value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                int i = indexMap[edge->circuitElement->name];
                mna[vminus][i] += -1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -edge->circuitElement->value;
                mna[i][vxminus] += edge->circuitElement->value;
            }
        }
        // When target node is connected to ground
        else if (edge->circuitElement->nodeB.compare("0") == 0) {
            int vplus = indexMap[edge->circuitElement->nodeA];
            // When controlling_element's source node is connected to ground
            if (edge->circuitElement->controlling_element->nodeA == "0") {
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                int i = indexMap[edge->circuitElement->name];
                mna[vplus][i] += 1.0;
                mna[i][vplus] += 1.0;
                mna[i][vxminus] += edge->circuitElement->value;
            }
            // When controlling_element's target node is connected to ground
            else if (edge->circuitElement->controlling_element->nodeB == "0") {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                int i = indexMap[edge->circuitElement->name];
                mna[vplus][i] += 1.0;
                mna[i][vplus] += 1.0;
                mna[i][vxplus] += -edge->circuitElement->value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                int i = indexMap[edge->circuitElement->name];
                mna[vplus][i] += 1.0;
                mna[i][vplus] += 1.0;
                mna[i][vxplus] += -edge->circuitElement->value;
                mna[i][vxminus] += edge->circuitElement->value;
            }
        }
        // When both the nodes are not connected ground
        else {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int vminus = indexMap[edge->circuitElement->nodeB];
            // When controlling_element's source node is connected to ground
            if (edge->circuitElement->controlling_element->nodeA == "0") {
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                int i = indexMap[edge->circuitElement->name];
                mna[vplus][i] += 1.0;
                mna[vminus][i] += -1.0;
                mna[i][vplus] += 1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxminus] += edge->circuitElement->value;
            }
            // When controlling_element's target node is connected to ground
            else if (edge->circuitElement->controlling_element->nodeB == "0") {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                int i = indexMap[edge->circuitElement->name];
                mna[vplus][i] += 1.0;
                mna[vminus][i] += -1.0;
                mna[i][vplus] += 1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -edge->circuitElement->value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                int i = indexMap[edge->circuitElement->name];
                mna[vplus][i] += 1.0;
                mna[vminus][i] += -1.0;
                mna[i][vplus] += 1.0;
                mna[i][vminus] += -1.0;
                mna[i][vxplus] += -edge->circuitElement->value;
                mna[i][vxminus] += edge->circuitElement->value;
            }
        }
    }
}

void Node::handleDepCurrentSource(const std::shared_ptr<Edge> &edge,
                                  std::map<std::string, int> &indexMap,
                                  std::vector<std::vector<double>> &mna,
                                  std::vector<double> &rhs)
{
    // Dependant Current Source (always Group 1)
    // Current Controlled Current Source (CCCS)
    if (edge->circuitElement->controlling_variable == ControlVariable::i) {
        if (edge->circuitElement->nodeA.compare("0") == 0) {
            int vminus = indexMap[edge->circuitElement->nodeB];
            int i = indexMap[edge->circuitElement->controlling_element->name];
            mna[vminus][i] += -edge->circuitElement->value;
        } else if (edge->circuitElement->nodeB.compare("0") == 0) {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int i = indexMap[edge->circuitElement->controlling_element->name];
            mna[vplus][i] += edge->circuitElement->value;
        } else {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int vminus = indexMap[edge->circuitElement->nodeB];
            int i = indexMap[edge->circuitElement->controlling_element->name];
            mna[vplus][i] += edge->circuitElement->value;
            mna[vminus][i] += -edge->circuitElement->value;
        }
    }
    // Voltage Controlled Current Source (VCCS)
    else {
        if (edge->circuitElement->nodeA.compare("0") == 0) {
            int vminus = indexMap[edge->circuitElement->nodeB];
            // When controlling_element's source node is connected to ground
            if (edge->circuitElement->controlling_element->nodeA == "0") {
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                mna[vminus][vxminus] += edge->circuitElement->value;
            }
            // When controlling_element's target node is connected to ground
            else if (edge->circuitElement->controlling_element->nodeB == "0") {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                mna[vminus][vxplus] += -edge->circuitElement->value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                mna[vminus][vxplus] += -edge->circuitElement->value;
                mna[vminus][vxminus] += edge->circuitElement->value;
            }
        }
        // When target node is connected to ground
        else if (edge->circuitElement->nodeB.compare("0") == 0) {
            int vplus = indexMap[edge->circuitElement->nodeA];
            // When controlling_element's source node is connected to ground
            if (edge->circuitElement->controlling_element->nodeA == "0") {
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                mna[vplus][vxminus] += -edge->circuitElement->value;
            }
            // When controlling_element's target node is connected to ground
            else if (edge->circuitElement->controlling_element->nodeB == "0") {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                mna[vplus][vxplus] += edge->circuitElement->value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                mna[vplus][vxplus] += edge->circuitElement->value;
                mna[vplus][vxminus] += -edge->circuitElement->value;
            }
        }
        // When both the nodes are not connected ground
        else {
            int vplus = indexMap[edge->circuitElement->nodeA];
            int vminus = indexMap[edge->circuitElement->nodeB];
            // When controlling_element's source node is connected to ground
            if (edge->circuitElement->controlling_element->nodeA == "0") {
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                mna[vplus][vxminus] += -edge->circuitElement->value;
                mna[vminus][vxminus] += edge->circuitElement->value;
            }
            // When controlling_element's target node is connected to ground
            else if (edge->circuitElement->controlling_element->nodeB == "0") {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                mna[vplus][vxplus] += edge->circuitElement->value;
                mna[vminus][vxplus] += -edge->circuitElement->value;
            }
            // When controlling_element's both the nodes are not connected
            // ground
            else {
                int vxplus =
                    indexMap[edge->circuitElement->controlling_element->nodeA];
                int vxminus =
                    indexMap[edge->circuitElement->controlling_element->nodeB];
                mna[vplus][vxplus] += edge->circuitElement->value;
                mna[vplus][vxminus] += -edge->circuitElement->value;
                mna[vminus][vxplus] += -edge->circuitElement->value;
                mna[vminus][vxminus] += edge->circuitElement->value;
            }
        }
    }
}
