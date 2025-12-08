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

#include <memory>
#include <string>
#include <vector>

#include "CircuitElement.hpp"

// Forward declare Parser used in the parse factory method.
class Parser;

/**
 * @file Capacitor.hpp
 * @brief Declaration of the linear Capacitor element.
 *
 * This header declares the `Capacitor` class which models an ideal linear
 * capacitor for both DC and transient (time-domain) simulations. The class
 * derives from `CircuitElement` and implements the interfaces required by the
 * simulator for stamping (DC and transient), transient companion computation,
 * parsing from a netlist, and updating internal state from the solver's
 * solution vector.
 *
 * Transient companion (trapezoidal rule) conventions used:
 *   Geq = 2 * C / h
 *   Ieq = Geq * v_n - i_n
 *
 * where `h` is the timestep and `v_n`, `i_n` are the stored previous-step
 * capacitor voltage and current respectively.
 */

/**
 * @class Capacitor
 * @brief Represents an ideal linear capacitor element.
 *
 * The `Capacitor` stores the minimal internal state required for transient
 * integration (previous voltage and current) and the per-step companion
 * parameters (`Geq`, `Ieq`) used to stamp a Norton equivalent during time-step
 * assembly.
 */
class Capacitor : public CircuitElement
{
   private:
    /**
     * @brief Voltage across the capacitor at the previous timestep (v_n).
     *
     * Units: Volts.
     */
    double v_prev = 0.0;

    /**
     * @brief Current through the capacitor at the previous timestep (i_n).
     *
     * Units: Amperes.
     */
    double i_prev = 0.0;

    /**
     * @brief Equivalent conductance for the trapezoidal companion (Geq).
     *
     * Geq = 2 * C / h. Units: Siemens.
     */
    double Geq = 0.0;

    /**
     * @brief Equivalent Norton current for the trapezoidal companion (Ieq).
     *
     * Ieq = Geq * v_n - i_n. Signed such that the companion current is
     * injected from `nodeA` -> `nodeB` when applied to the RHS.
     *
     * Units: Amperes.
     */
    double Ieq = 0.0;

   public:
    /**
     * @brief Construct a new Capacitor element.
     *
     * @param name Element identifier (e.g., "C1").
     * @param nodeA Positive node name (netlist token 1).
     * @param nodeB Negative node name (netlist token 2).
     * @param value Capacitance in Farads (netlist token 3). Must be non-zero.
     */
    Capacitor(const std::string& name, const std::string& nodeA,
              const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }

    /**
     * @brief Stamp the element for DC/MNA assembly.
     *
     * For an ideal capacitor this is typically a no-op for DC operating-point
     * analysis (open-circuit). The transient solver uses `computeCompanion` and
     * `stampTransient` for time-domain assembly.
     *
     * @param mna Modified Nodal Analysis matrix (NxN).
     * @param rhs Right-hand side vector (N).
     * @param indexMap Map from node names to indices in `mna`/`rhs`. Node name
     *                 "0" denotes ground.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parse a capacitor from tokenized netlist input.
     *
     * Expected token formats:
     *   Cname nodeA nodeB value
     * Optionally a group token may be present depending on parser rules.
     *
     * On success returns a `std::shared_ptr<CircuitElement>` pointing to the
     * created `Capacitor`. On failure returns `nullptr` and emits diagnostics.
     *
     * @param parser Reference to the Parser helper for validation and parsing.
     * @param tokens Tokenized netlist line.
     * @param lineNumber Line number in the netlist (for diagnostics).
     * @return std::shared_ptr<CircuitElement> Created capacitor or nullptr on
     * error.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);

    /**
     * @brief Compute trapezoidal-rule companion parameters for timestep `h`.
     *
     * Computes and stores `Geq` and `Ieq` for use by `stampTransient`.
     * Behavior for non-positive `h`: `Geq` and `Ieq` are set to zero to avoid
     * division by zero.
     *
     * @param h Time-step size in seconds. Must be > 0 for meaningful values.
     */
    void computeCompanion(double h) override;

    /**
     * @brief Stamp the transient Norton companion into `mna` and `rhs`.
     *
     * Precondition: `computeCompanion(h)` must be called to set `Geq` and
     * `Ieq` for the current timestep. Stamps conductance `Geq` between the two
     * capacitor nodes and injects `Ieq` into `rhs` with the convention that
     * `Ieq` is the current from `nodeA` -> `nodeB`.
     *
     * @param mna Modified Nodal Analysis matrix to modify.
     * @param rhs RHS vector to modify.
     * @param indexMap Map from node names to indices in `mna`/`rhs`. "0" is
     * ground.
     */
    void stampTransient(std::vector<std::vector<double>>& mna,
                        std::vector<double>& rhs,
                        std::map<std::string, int>& indexMap) override;

    /**
     * @brief Update internal state from the solver's solution vector.
     *
     * Reads the node voltages from `x` (using `indexMap`), updates `v_prev` to
     * the capacitor voltage at the new timestep and sets `i_prev` using the
     * companion relation:
     *   i_{n+1} = Geq * v_{n+1} - Ieq
     *
     * @param x Solution vector (Eigen::VectorXd compatible Ref).
     * @param indexMap Map from node names to indices in `x`.
     */
    void updateStateFromSolution(
        const Eigen::Ref<const Eigen::VectorXd>& x,
        const std::map<std::string, int>& indexMap) override;
};
