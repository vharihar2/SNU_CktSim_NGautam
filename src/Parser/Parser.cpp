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
 * @file Parser.cpp
 * @brief Implementation of the Parser declared in Parser.hpp.
 *
 * This file contains the concrete implementation of the `Parser` class.
 * The parser uses a two-pass approach:
 *
 *  1. Read the netlist, collect `.SUBCKT` definitions and top-level lines.
 *  2. Expand hierarchical instances (lines beginning with `X...`) into a
 *     flattened set of element lines and then parse those lines into
 *     concrete element objects (e.g., `Resistor`, `Capacitor`,
 * `VoltageSource`).
 *
 * Implementation notes (concise):
 *  - Tokenization preserves parenthesized groups (so expressions like
 *    SIN(...) remain intact) and uppercases text for consistent keyword
 *    recognition while leaving punctuation unchanged.
 *  - Subcircuit expansion performs instance-qualified name mangling using
 *    the `<element>:<instancePath>` convention to avoid name collisions.
 *  - Numeric parsing follows strict SPICE-like rules: common suffixes
 *    (T, G, MEG, K, M, U, N, P, F) are supported and the mantissa must be
 *    a well-formed floating literal (std::stod must consume the entire
 * mantissa).
 *
 * Keep API-level documentation in the header (`Parser.hpp`). This file
 * contains only concise implementation-level documentation and targeted
 * inline comments to aid future maintainers.
 */

#include "Parser.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <ostream>
#include <sstream>
#include <unordered_map>

#include "Capacitor.hpp"
#include "CircuitElement.hpp"
#include "CurrentSource.hpp"
#include "DependentCurrentSource.hpp"
#include "DependentVoltageSource.hpp"
#include "Inductor.hpp"
#include "NonlinearCapacitor.hpp"
#include "NonlinearInductor.hpp"
#include "Resistor.hpp"
#include "Solver.hpp"
#include "VoltageSource.hpp"

int Parser::parse(const std::string& fileName, SolverDirectiveType& directive)
{
    std::ifstream fileStream(fileName);
    if (!fileStream) {
        std::cerr << "Error: Netlist not available in the project directory"
                  << std::endl;
        return 1;
    }

    std::string line;
    int lineNumber = 1;
    int errorCount = 0;

    // Collect top-level non-subckt lines for later expansion
    std::vector<std::string> topLevelLines;

    // Clear previous data
    circuitElements.clear();
    nodes_group2.clear();
    elementMap.clear();

    // Helper tokenizer: splits on whitespace but keeps parenthesized groups
    auto tokenizeLine = [](const std::string& ln) {
        std::vector<std::string> tokens;
        std::string cur;
        bool inParen = false;
        int parenDepth = 0;
        for (size_t i = 0; i < ln.size(); ++i) {
            char c = ln[i];
            // If start of comment outside parentheses, stop tokenizing
            if (!inParen && (c == '*' || c == ';')) break;
            if (std::isspace((unsigned char)c) && !inParen) {
                if (!cur.empty()) {
                    tokens.push_back(cur);
                    cur.clear();
                }
                continue;
            }
            cur.push_back(c);
            if (c == '(') {
                inParen = true;
                ++parenDepth;
            } else if (c == ')' && inParen) {
                if (--parenDepth == 0) inParen = false;
            }
        }
        if (!cur.empty()) tokens.push_back(cur);
        return tokens;
    };

    while (std::getline(fileStream, line)) {
        lineNumber++;

        // Skip empty or comment-only lines quickly by checking first non-space
        size_t firstChar = line.find_first_not_of(" \t\r\n");
        if (firstChar == std::string::npos) continue;  // blank line
        char fc = line[firstChar];
        if (fc == '*' || fc == ';') continue;  // comment line

        // Convert line to uppercase for uniformity (keeps punctuation like ()
        // unchanged)
        std::string up = line;
        std::transform(up.begin(), up.end(), up.begin(), ::toupper);

        // Tokenize line using the tokenizer that preserves (...) groups
        std::vector<std::string> tokens = tokenizeLine(up);

        // Skip empty token lines
        if (tokens.empty()) continue;

        // Handle explicit global end directive (.END). Do NOT treat .ENDS
        // (subcircuit end) as a global terminator; .ENDS is handled while
        // collecting subcircuit bodies below.
        if (tokens[0] == ".END") break;

        // Collect subcircuit definitions: .SUBCKT <name> <port1> <port2> ...
        if (tokens[0].find(".SUBCKT") == 0) {
            // Validate presence of a name
            if (tokens.size() < 2) {
                std::cerr << "Line " << lineNumber
                          << ": .SUBCKT requires a name" << std::endl;
                ++errorCount;
                continue;
            }
            std::string subName = tokens[1];
            if (subcktDefs.find(subName) != subcktDefs.end()) {
                std::cerr << "Line " << lineNumber
                          << ": Duplicate .SUBCKT name '" << subName << "'"
                          << std::endl;
                ++errorCount;
                continue;
            }
            Parser::SubcktDef def;
            def.name = subName;
            def.definitionLine = lineNumber;
            // ports are tokens[2] .. end
            for (size_t i = 2; i < tokens.size(); ++i)
                def.ports.push_back(tokens[i]);

            // Collect body lines until matching .ENDS (or EOF)
            std::string bodyLine;
            bool foundEnds = false;
            while (std::getline(fileStream, bodyLine)) {
                ++lineNumber;
                size_t fc = bodyLine.find_first_not_of(" \t\r\n");
                if (fc == std::string::npos) continue;
                char fch = bodyLine[fc];
                if (fch == '*' || fch == ';') continue;  // comment-only

                std::string upBody = bodyLine;
                std::transform(upBody.begin(), upBody.end(), upBody.begin(),
                               ::toupper);
                // Tokenize the body line to quickly check for .ENDS
                std::vector<std::string> bodyTokens = tokenizeLine(upBody);
                if (!bodyTokens.empty() && bodyTokens[0] == ".ENDS") {
                    foundEnds = true;
                    break;
                }
                // Store the raw uppercased body line for later expansion
                def.body.push_back(upBody);
            }
            if (!foundEnds) {
                std::cerr << "Line " << def.definitionLine << ": .SUBCKT '"
                          << def.name << "' missing matching .ENDS"
                          << std::endl;
                ++errorCount;
            }
            // Save def even if missing .ENDS to allow error reporting later
            subcktDefs[def.name] = def;
            continue;
        }

        // Top-level handling: capture directives and store other lines for
        // expansion
        if (tokens[0].find(".OP") == 0) {
            // Operational point directive
            if (directive != SolverDirectiveType::NONE) {
                std::cerr << "Warning: Multiple directives found. Using the "
                             "first one."
                          << std::endl;
                ++errorCount;
            }
            directive = SolverDirectiveType::OPERATING_POINT;
            continue;
        }

        if (tokens[0].find(".TRAN") == 0) {
            // Transient directive: .TRAN <Tstep> <Tstop>
            if (directive != SolverDirectiveType::NONE) {
                std::cerr << "Warning: Multiple directives found. Using the "
                             "first one."
                          << std::endl;
                ++errorCount;
            }
            // Defer numeric parsing/storage until after expansion
            topLevelLines.push_back(up);
            continue;
        }

        // Otherwise treat as a top-level element line and store for expansion
        topLevelLines.push_back(up);
    }

    // Second pass: expand X instances and parse expanded lines into elements
    std::vector<std::string> expandedLines;

    // Helper: simple numeric detector
    auto isNumberLike = [](const std::string& s) {
        if (s.empty()) return false;
        try {
            std::size_t idx;
            std::stod(s, &idx);
            return idx == s.size();
        } catch (...) {
            return false;
        }
    };

    // Keywords to avoid suffixing
    std::set<std::string> keywords = {"POLY", "SIN", "PULSE",
                                      "DC",   "AC",  "VALUE"};

    std::function<void(const std::vector<std::string>&, const std::string&,
                       int)>
        expandLines;
    expandLines = [&](const std::vector<std::string>& lines,
                      const std::string& parentPath, int depth) {
        if (depth > subcktRecursionLimit) {
            std::cerr << "Error: Subcircuit recursion limit ("
                      << subcktRecursionLimit << ") exceeded while expanding '"
                      << parentPath << "'" << std::endl;
            ++errorCount;
            return;
        }
        for (const auto& raw : lines) {
            std::vector<std::string> toks = tokenizeLine(raw);
            if (toks.empty()) continue;
            if (toks[0].size() > 0 && toks[0][0] == 'X') {
                // instance: Xname <nets...> subcktName
                if (toks.size() < 2) {
                    std::cerr << "Error: Malformed subckt instance: '" << raw
                              << "'" << std::endl;
                    ++errorCount;
                    continue;
                }
                std::string instName = toks[0];
                std::string subName = toks.back();
                std::vector<std::string> nets;
                if (toks.size() >= 3) {
                    for (size_t i = 1; i + 1 < toks.size(); ++i)
                        nets.push_back(toks[i]);
                }
                auto it = subcktDefs.find(subName);
                if (it == subcktDefs.end()) {
                    std::cerr << "Error: Unknown subcircuit '" << subName
                              << "' referenced by instance '" << instName << "'"
                              << std::endl;
                    ++errorCount;
                    continue;
                }
                const Parser::SubcktDef& def = it->second;
                if (def.ports.size() != nets.size()) {
                    std::cerr << "Error: Port count mismatch for instance '"
                              << instName << "' of '" << subName
                              << "' (expected " << def.ports.size() << ", got "
                              << nets.size() << ")" << std::endl;
                    ++errorCount;
                    continue;
                }
                std::string instancePath =
                    parentPath.empty() ? instName : parentPath + "." + instName;
                std::vector<std::string> substituted;
                for (const auto& bodyLine : def.body) {
                    std::vector<std::string> btoks = tokenizeLine(bodyLine);
                    if (btoks.empty()) continue;
                    std::vector<std::string> ntoks;
                    std::string newName = btoks[0] + ":" + instancePath;
                    ntoks.push_back(newName);
                    for (size_t i = 1; i < btoks.size(); ++i) {
                        const std::string& tok = btoks[i];
                        if (tok == "0") {
                            ntoks.push_back(tok);
                            continue;
                        }
                        bool replaced = false;
                        for (size_t p = 0; p < def.ports.size(); ++p) {
                            if (tok == def.ports[p]) {
                                ntoks.push_back(nets[p]);
                                replaced = true;
                                break;
                            }
                        }
                        if (replaced) continue;
                        if (isNumberLike(tok) ||
                            keywords.find(tok) != keywords.end()) {
                            ntoks.push_back(tok);
                            continue;
                        }
                        bool shouldSuffix = false;
                        if (tok.find_first_of("0123456789") !=
                            std::string::npos)
                            shouldSuffix = true;
                        else if (tok.size() > 1 &&
                                 std::isalpha((unsigned char)tok[0]) &&
                                 std::isdigit((unsigned char)tok[1]))
                            shouldSuffix = true;
                        if (shouldSuffix)
                            ntoks.push_back(tok + ":" + instancePath);
                        else
                            ntoks.push_back(tok);
                    }
                    std::ostringstream oss;
                    for (size_t i = 0; i < ntoks.size(); ++i) {
                        if (i) oss << ' ';
                        oss << ntoks[i];
                    }
                    substituted.push_back(oss.str());
                }
                expandLines(substituted, instancePath, depth + 1);
            } else {
                if (parentPath.empty()) {
                    expandedLines.push_back(raw);
                } else {
                    std::vector<std::string> btoks = tokenizeLine(raw);
                    if (btoks.empty()) continue;
                    std::vector<std::string> ntoks;
                    ntoks.push_back(btoks[0] + ":" + parentPath);
                    for (size_t i = 1; i < btoks.size(); ++i) {
                        const std::string& tok = btoks[i];
                        if (tok == "0") {
                            ntoks.push_back(tok);
                            continue;
                        }
                        if (isNumberLike(tok) ||
                            keywords.find(tok) != keywords.end()) {
                            ntoks.push_back(tok);
                            continue;
                        }
                        bool shouldSuffix = false;
                        if (tok.find_first_of("0123456789") !=
                            std::string::npos)
                            shouldSuffix = true;
                        else if (tok.size() > 1 &&
                                 std::isalpha((unsigned char)tok[0]) &&
                                 std::isdigit((unsigned char)tok[1]))
                            shouldSuffix = true;
                        if (shouldSuffix)
                            ntoks.push_back(tok + ":" + parentPath);
                        else
                            ntoks.push_back(tok);
                    }
                    std::ostringstream oss;
                    for (size_t i = 0; i < ntoks.size(); ++i) {
                        if (i) oss << ' ';
                        oss << ntoks[i];
                    }
                    expandedLines.push_back(oss.str());
                }
            }
        }
    };

    expandLines(topLevelLines, "", 0);

    // Parse expanded lines into elements
    int virtualLineNumber = 1;
    for (const auto& up : expandedLines) {
        ++virtualLineNumber;
        std::vector<std::string> tokens = tokenizeLine(up);
        if (tokens.empty()) continue;

        // Handle directives in expanded lines
        if (tokens[0].find(".OP") == 0) {
            if (directive != SolverDirectiveType::NONE) {
                std::cerr << "Warning: Multiple directives found. Using the "
                             "first one."
                          << std::endl;
                ++errorCount;
            }
            directive = SolverDirectiveType::OPERATING_POINT;
            continue;
        }
        if (tokens[0].find(".TRAN") == 0) {
            if (directive != SolverDirectiveType::NONE) {
                std::cerr << "Warning: Multiple directives found. Using the "
                             "first one."
                          << std::endl;
                ++errorCount;
            }
            if (tokens.size() != 3) {
                std::cerr << "Line " << virtualLineNumber
                          << ": .TRAN expects 2 arguments: <Tstep> <Tstop>"
                          << std::endl;
                ++errorCount;
            } else {
                bool validStep = false;
                bool validStop = false;
                double step =
                    parseValue(tokens[1], virtualLineNumber, validStep);
                double stop =
                    parseValue(tokens[2], virtualLineNumber, validStop);
                if (!validStep || !validStop) {
                    std::cerr << "Line " << virtualLineNumber
                              << ": Invalid numeric argument in .TRAN directive"
                              << std::endl;
                    ++errorCount;
                } else if (step <= 0.0 || stop <= 0.0) {
                    std::cerr << "Line " << virtualLineNumber
                              << ": .TRAN arguments must be positive numbers"
                              << std::endl;
                    ++errorCount;
                } else if (step > stop) {
                    std::cerr << "Line " << virtualLineNumber
                              << ": .TRAN step must be <= stop time"
                              << std::endl;
                    ++errorCount;
                } else {
                    this->tranStep = step;
                    this->tranStop = stop;
                    directive = SolverDirectiveType::TRANSIENT;
                }
            }
            continue;
        }

        std::shared_ptr<CircuitElement> element = nullptr;
        if (tokens[0].find("R") == 0) {
            ++elementCounts.resistorCount;
            element = Resistor::parse(*this, tokens, virtualLineNumber);
            if (element && element->getGroup() == Group::G2)
                nodes_group2.insert(element->getName());
        } else if (tokens[0].find("V") == 0 && tokens[0].find("VC") != 0) {
            ++elementCounts.voltageSourceCount;
            element = VoltageSource::parse(*this, tokens, virtualLineNumber);
            if (element) nodes_group2.insert(element->getName());
        } else if (tokens[0].find("I") == 0 && tokens[0].find("IC") != 0) {
            ++elementCounts.currentSourceCount;
            element = CurrentSource::parse(*this, tokens, virtualLineNumber);
        } else if (tokens[0].find("C") == 0) {
            ++elementCounts.capacitorCount;
            if (tokens.size() >= 5 && tokens[4] == "POLY") {
                bool validValue = false;
                double value =
                    parseValue(tokens[3], virtualLineNumber, validValue);
                if (!validValue) {
                    std::cerr << "Line " << virtualLineNumber
                              << ": Invalid capacitor value" << std::endl;
                    ++errorCount;
                } else {
                    std::vector<double> coeffs;
                    for (size_t i = 5; i < tokens.size(); ++i) {
                        bool ok = false;
                        double c = parseValue(tokens[i], virtualLineNumber, ok);
                        if (!ok) {
                            std::cerr << "Line " << virtualLineNumber
                                      << ": Invalid polynomial coefficient '"
                                      << tokens[i] << "'" << std::endl;
                            ++errorCount;
                            break;
                        }
                        coeffs.push_back(c);
                    }
                    if (!coeffs.empty()) {
                        auto model = makePolynomialChargeModel(coeffs);
                        element = std::make_shared<NonlinearCapacitor>(
                            tokens[0], tokens[1], tokens[2], value, model);
                    } else {
                        std::cerr << "Line " << virtualLineNumber
                                  << ": POLY specified but no coefficients"
                                  << std::endl;
                        ++errorCount;
                    }
                }
            } else {
                element = Capacitor::parse(*this, tokens, virtualLineNumber);
            }
        } else if (tokens[0].find("L") == 0) {
            ++elementCounts.inductorCount;
            if (tokens.size() >= 5 && tokens[4] == "POLY") {
                bool validValue = false;
                double value =
                    parseValue(tokens[3], virtualLineNumber, validValue);
                if (!validValue) {
                    std::cerr << "Line " << virtualLineNumber
                              << ": Invalid inductor value" << std::endl;
                    ++errorCount;
                } else {
                    std::vector<double> coeffs;
                    for (size_t i = 5; i < tokens.size(); ++i) {
                        bool ok = false;
                        double c = parseValue(tokens[i], virtualLineNumber, ok);
                        if (!ok) {
                            std::cerr << "Line " << virtualLineNumber
                                      << ": Invalid polynomial coefficient '"
                                      << tokens[i] << "'" << std::endl;
                            ++errorCount;
                            break;
                        }
                        coeffs.push_back(c);
                    }
                    if (!coeffs.empty()) {
                        auto model = makePolynomialFluxModel(coeffs);
                        element = std::make_shared<NonlinearInductor>(
                            tokens[0], tokens[1], tokens[2], value, model);
                        nodes_group2.insert(element->getName());
                    } else {
                        std::cerr << "Line " << virtualLineNumber
                                  << ": POLY specified but no coefficients"
                                  << std::endl;
                        ++errorCount;
                    }
                }
            } else {
                element = Inductor::parse(*this, tokens, virtualLineNumber);
                if (element) nodes_group2.insert(element->getName());
            }
        } else if (tokens[0].find("VC") == 0) {
            ++elementCounts.depVoltageSourceCount;
            element =
                DependentVoltageSource::parse(*this, tokens, virtualLineNumber);
            if (element) nodes_group2.insert(element->getName());
        } else if (tokens[0].find("IC") == 0) {
            ++elementCounts.depCurrentSourceCount;
            element =
                DependentCurrentSource::parse(*this, tokens, virtualLineNumber);
        } else {
            std::cerr
                << "Error: Unknown element after expansion at virtual line "
                << virtualLineNumber << ": " << up << std::endl;
            ++errorCount;
        }

        if (element) {
            nodes_group2.insert(element->getNodeA());
            nodes_group2.insert(element->getNodeB());
            circuitElements.push_back(element);
            elementMap[element->getName()] = element;
        }
    }

    // Resolve controlling_element pointers for dependent sources
    // for (auto& elem : circuitElements) {
    //     if (elem->controlling_variable != ControlVariable::none &&
    //         elem->controlling_element) {
    //         auto it = elementMap.find(elem->controlling_element->name);
    //         if (it != elementMap.end()) {
    //             elem->controlling_element = it->second;
    //             // If controlling variable is current, referenced element
    //             must
    //             // be group 2
    //             if (elem->controlling_variable == ControlVariable::i &&
    //                 elem->controlling_element->group != Group::G2) {
    //                 std::cerr << "Warning: Referenced element "
    //                           << elem->controlling_element->name
    //                           << " must be in group 2 as its current
    //                           variable
    //                           "
    //                              "is required by "
    //                           << elem->name << std::endl;
    //                 elem->controlling_element->group = Group::G2;
    //                 nodes_group2.insert(elem->controlling_element->name);
    //             }
    //         } else {
    //             std::cerr << "Error: Referenced element "
    //                       << elem->controlling_element->name
    //                       << " (referenced by " << elem->name
    //                       << ") is not present in the netlist" <<
    //                       std::endl;
    //             errorCount++;
    //         }
    //     }
    // }

    errorCount += CircuitElement::resolveDependencies(circuitElements,
                                                      elementMap, nodes_group2);
    // Check for ground node
    if (nodes_group2.find("0") == nodes_group2.end()) {
        std::cerr << "Error: Circuit must contain ground (0)" << std::endl;
        errorCount++;
    }

    // Print summary (optional)
    std::cout << "\nTotal elements in the circuit: " << circuitElements.size()
              << std::endl;

    printElementCounts();
    addStabilityResistors();
    return errorCount;
}

void Parser::printParser()
{
    // for (std::shared_ptr<CircuitElement> circuitElement :
    // circuitElements)
    //     if (circuitElement->type == ElementType::V ||
    //         circuitElement->type == ElementType::I ||
    //         circuitElement->type == ElementType::R)
    //         cout << circuitElement->name + " " + circuitElement->nodeA +
    //         " "
    //         +
    //                     circuitElement->nodeB + " "
    //              << circuitElement->value << " " << circuitElement->group
    //              << endl;
    //     else
    //         cout << circuitElement->name + " " + circuitElement->nodeA +
    //         " "
    //         +
    //                     circuitElement->nodeB + " "
    //              << circuitElement->value << " " << circuitElement->group
    //              << " "
    //              << circuitElement->controlling_variable << " "
    //              << circuitElement->controlling_element->name + " " +
    //                     circuitElement->controlling_element->nodeA + " "
    //                     + circuitElement->controlling_element->nodeB
    //              << endl;
}
bool Parser::validateTokens(const std::vector<std::string>& tokens,
                            int expectedSize, int lineNumber)
{
    if (tokens.size() != expectedSize) {
        std::cerr << "Line " << lineNumber << ": Expected " << expectedSize
                  << " tokens, got " << tokens.size() << std::endl;
        return false;
    }
    return true;
}

bool Parser::validateNodes(const std::string& nodeA, const std::string& nodeB,
                           int lineNumber)
{
    if (nodeA == nodeB) {
        std::cerr << "Line " << lineNumber
                  << ": NodeA and NodeB cannot be the same (" << nodeA << ")"
                  << std::endl;
        return false;
    }
    return true;
}

double Parser::parseValue(const std::string& valueStr, int lineNumber,
                          bool& valid)
{
    // Strict SPICE-style parsing:
    // - Recognize common suffixes (T, G, MEG, K, M, U, N, P, F).
    // - Require a non-empty mantissa.
    // - Require that the mantissa is fully consumed by std::stod (no stray
    //   characters like extra '.' or trailing digits) — i.e., the numeric
    //   literal must be well-formed.
    // - If a suffix exists, it must be one of the recognized suffixes and the
    //   mantissa must also be a fully-formed numeric literal.
    //
    // This matches LTspice behavior where malformed numbers such as "1.2.3"
    // are rejected.

    if (valueStr.empty()) {
        std::cerr << "Line " << lineNumber << ": Invalid value '" << valueStr
                  << "'" << std::endl;
        valid = false;
        return 0.0;
    }

    // Map of recognized suffixes (uppercase) -> multiplier
    static const std::unordered_map<std::string, double> suffixMap = {
        {"T", 1e12}, {"G", 1e9},  {"MEG", 1e6}, {"K", 1e3},  {"M", 1e-3},
        {"U", 1e-6}, {"N", 1e-9}, {"P", 1e-12}, {"F", 1e-15}};

    // Separate trailing alphabetic suffix (if any). We accept up to 3 letters
    // (to support MEG). valueStr is uppercased by the caller earlier in the
    // pipeline; be defensive and uppercase suffix here too.
    size_t pos = valueStr.size();
    while (pos > 0 && std::isalpha((unsigned char)valueStr[pos - 1])) --pos;

    std::string mantissa = valueStr.substr(0, pos);
    std::string suffix = valueStr.substr(pos);

    // Mantissa must not be empty (e.g., "K" is invalid)
    if (mantissa.empty()) {
        std::cerr << "Line " << lineNumber << ": Invalid value '" << valueStr
                  << "'" << std::endl;
        valid = false;
        return 0.0;
    }

    // Normalize suffix to uppercase (defensive)
    for (auto& c : suffix) c = (char)std::toupper((unsigned char)c);

    // Helper to parse a mantissa and require full consumption of the string
    auto parseMantissaStrict = [&](const std::string& m, double& out) -> bool {
        try {
            size_t idx = 0;
            out = std::stod(m, &idx);
            // require std::stod consumed the whole mantissa string
            if (idx != m.size()) {
                return false;
            }
            return true;
        } catch (const std::exception&) {
            return false;
        }
    };

    // If no suffix, parse mantissa strictly
    if (suffix.empty()) {
        double value = 0.0;
        if (!parseMantissaStrict(mantissa, value)) {
            std::cerr << "Line " << lineNumber << ": Invalid value '"
                      << valueStr << "'" << std::endl;
            valid = false;
            return 0.0;
        }
        valid = true;
        return value;
    }

    // Suffix present: must be recognized
    auto it = suffixMap.find(suffix);
    if (it == suffixMap.end()) {
        std::cerr << "Line " << lineNumber << ": Unknown suffix '" << suffix
                  << "' in value '" << valueStr << "'" << std::endl;
        valid = false;
        return 0.0;
    }

    // Parse mantissa strictly, then scale
    double base = 0.0;
    if (!parseMantissaStrict(mantissa, base)) {
        std::cerr << "Line " << lineNumber << ": Invalid numeric part '"
                  << mantissa << "' in '" << valueStr << "'" << std::endl;
        valid = false;
        return 0.0;
    }

    double value = base * it->second;
    valid = true;
    return value;
}

// void Parser::parseResistor(const std::vector<std::string>& tokens,
//                            int lineNumber)
// {
//     // Validate token count
//     if (!validateTokens(tokens, 4, lineNumber) &&
//         !validateTokens(tokens, 5, lineNumber)) {
//         std::cerr << "Error: Invalid resistor definition at line " <<
//         lineNumber
//                   << std::endl;
//         return;
//     }
//
//     // Validate nodes
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Resistor nodes cannot be the same at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     // Parse value
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for resistor value at line
//         "
//                   << lineNumber << std::endl;
//         value = 1;  // Default to 1 ohm if invalid
//     }
//
//     // Create resistor element
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::R;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     // Update nodes_group2 if group is G2
//     if (temp->group == Group::G2) {
//         nodes_group2.insert(temp->name);
//     }
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     // Add to circuitElements and elementMap
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
//
// void Parser::parseVoltageSource(const std::vector<std::string>& tokens,
//                                 int lineNumber)
// {
//     // Validate token count
//     if (!validateTokens(tokens, 4, lineNumber)) {
//         std::cerr << "Error: Invalid voltage source definition at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     // Validate nodes
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Voltage source nodes cannot be the same at
//         line
//         "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     // Parse value
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for voltage source value at
//         line "
//                   << lineNumber << std::endl;
//         value = 1;  // Default to 1V if invalid
//     }
//
//     // Create voltage source element
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::V;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     // Update nodes_group2
//     nodes_group2.insert(temp->name);
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     // Add to circuitElements and elementMap
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
//
// void Parser::parseCurrentSource(const std::vector<std::string>& tokens,
//                                 int lineNumber)
// {
//     // Valid token counts: 4 or 5 (for optional group)
//     if (!validateTokens(tokens, 4, lineNumber) &&
//         !validateTokens(tokens, 5, lineNumber)) {
//         std::cerr << "Error: Invalid current source definition at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Current source nodes cannot be the same at
//         line
//         "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for current source value at
//         line "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::I;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
// void Parser::parseCapacitor(const std::vector<std::string>& tokens,
//                             int lineNumber)
// {
//     // Valid token counts: 4 or 5 (for optional group)
//     if (!validateTokens(tokens, 4, lineNumber) &&
//         !validateTokens(tokens, 5, lineNumber)) {
//         std::cerr << "Error: Invalid capacitor definition at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Capacitor nodes cannot be the same at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for capacitor value at line
//         "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::C;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
// void Parser::parseInductor(const std::vector<std::string>& tokens,
//                            int lineNumber)
// {
//     // Inductors are always group 2
//     if (!validateTokens(tokens, 4, lineNumber)) {
//         std::cerr << "Error: Invalid inductor definition at line " <<
//         lineNumber
//                   << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Inductor nodes cannot be the same at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for inductor value at line
//         "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::L;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//     temp->controlling_variable = ControlVariable::none;
//     temp->controlling_element = nullptr;
//     temp->processed = false;
//
//     nodes_group2.insert(temp->name);
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
//     elementMap[temp->name] = temp;
// }
//
// void Parser::parseDependentVoltageSource(const std::vector<std::string>&
// tokens,
//                                          int lineNumber)
// {
//     // Must have at least 6 tokens: VC name nodeA nodeB value
//     // controlling_variable controlling_element
//     if (!validateTokens(tokens, 6, lineNumber)) {
//         std::cerr
//             << "Error: Invalid dependent voltage source definition at
//             line "
//             << lineNumber << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Dependent voltage source nodes cannot be the
//         same "
//                      "at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for dependent voltage
//         source "
//                      "value at line "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::Vc;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//
//     // Controlling variable: must be "V" or "I"
//     if (tokens[4] == "V") {
//         temp->controlling_variable = ControlVariable::v;
//     } else if (tokens[4] == "I") {
//         temp->controlling_variable = ControlVariable::i;
//     } else {
//         std::cerr << "Error: Illegal controlling variable argument at
//         line "
//                   << lineNumber << std::endl;
//         temp->controlling_variable = ControlVariable::none;
//     }
//
//     // Cascading check: controlling element cannot be VC or IC
//     if (tokens[5].find("IC") == 0 || tokens[5].find("VC") == 0) {
//         std::cerr << "Error: Controlled source " << tokens[0]
//                   << " cannot be cascaded at line " << lineNumber <<
//                   std::endl;
//         temp->controlling_variable = ControlVariable::none;
//         temp->controlling_element = nullptr;
//     } else {
//         temp->controlling_element = std::make_shared<CircuitElement>();
//         temp->controlling_element->name = tokens[5];
//     }
//
//     temp->processed = false;
//
//     nodes_group2.insert(temp->name);
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
// }
//
// void Parser::parseDependentCurrentSource(const std::vector<std::string>&
// tokens,
//                                          int lineNumber)
// {
//     // Must have at least 6 tokens: IC name nodeA nodeB value
//     // controlling_variable controlling_element
//     if (!validateTokens(tokens, 6, lineNumber)) {
//         std::cerr
//             << "Error: Invalid dependent current source definition at
//             line "
//             << lineNumber << std::endl;
//         return;
//     }
//
//     if (!validateNodes(tokens[1], tokens[2], lineNumber)) {
//         std::cerr << "Error: Dependent current source nodes cannot be the
//         same "
//                      "at line "
//                   << lineNumber << std::endl;
//         return;
//     }
//
//     bool validValue = false;
//     double value = parseValue(tokens[3], lineNumber, validValue);
//     if (!validValue || value == 0) {
//         std::cerr << "Error: Illegal argument for dependent current
//         source "
//                      "value at line "
//                   << lineNumber << std::endl;
//         value = 1;
//     }
//
//     auto temp = std::make_shared<CircuitElement>();
//     temp->name = tokens[0];
//     temp->type = ElementType::Ic;
//     temp->nodeA = tokens[1];
//     temp->nodeB = tokens[2];
//     temp->group = Group::G2;
//     temp->value = value;
//
//     // Controlling variable: must be "V" or "I"
//     if (tokens[4] == "V") {
//         temp->controlling_variable = ControlVariable::v;
//     } else if (tokens[4] == "I") {
//         temp->controlling_variable = ControlVariable::i;
//     } else {
//         std::cerr << "Error: Illegal controlling variable argument at
//         line "
//                   << lineNumber << std::endl;
//         temp->controlling_variable = ControlVariable::none;
//     }
//
//     // Cascading check: controlling element cannot be VC or IC
//     if (tokens[5].find("IC") == 0 || tokens[5].find("VC") == 0) {
//         std::cerr << "Error: Controlled source " << tokens[0]
//                   << " cannot be cascaded at line " << lineNumber <<
//                   std::endl;
//         temp->controlling_variable = ControlVariable::none;
//         temp->controlling_element = nullptr;
//     } else {
//         temp->controlling_element = std::make_shared<CircuitElement>();
//         temp->controlling_element->name = tokens[5];
//     }
//
//     temp->processed = false;
//
//     nodes_group2.insert(temp->nodeA);
//     nodes_group2.insert(temp->nodeB);
//
//     circuitElements.push_back(temp);
// }

void Parser::printElementCounts() const
{
    std::cout << "Total Resistors: " << elementCounts.resistorCount
              << std::endl;
    std::cout << "Total Voltage Sources: " << elementCounts.voltageSourceCount
              << std::endl;
    std::cout << "Total Current Sources: " << elementCounts.currentSourceCount
              << std::endl;
    std::cout << "Total Capacitors: " << elementCounts.capacitorCount
              << std::endl;
    std::cout << "Total Inductors: " << elementCounts.inductorCount
              << std::endl;
    std::cout << "Total Dependent Voltage Sources: "
              << elementCounts.depVoltageSourceCount << std::endl;
    std::cout << "Total Dependent Current Sources: "
              << elementCounts.depCurrentSourceCount << std::endl;
}

void Parser::addStabilityResistors()
{
    int stabilityResistorIndex = 1;
    for (const auto& node : nodes_group2) {
        if (node == "0") continue;  // Skip ground
        std::string resistorName = "RSTAB_" + node;
        auto temp = std::make_shared<Resistor>(resistorName, node, "0",
                                               STABILITY_RESISTOR_VALUE);
        circuitElements.push_back(temp);
        elementMap[temp->getName()] = temp;
        // Optionally: nodes_group2.insert(node); // Already present
        stabilityResistorIndex++;
    }
}
